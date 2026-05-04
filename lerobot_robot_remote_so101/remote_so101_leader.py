import json
import logging
import signal
import socket
import sys
import threading
import time

import numpy as np
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import RemoteSO101LeaderConfig

logger = logging.getLogger(__name__)


class RemoteSO101Leader(Teleoperator):
    """Remote SO101 Leader teleoperator that reads joint positions from an ESP32 over TCP.

    The Leader ESP32 streams normalized joint positions at ``LEADER_STREAM_HZ`` (30 Hz).
    This teleoperator receives the stream, caches the latest frame, and returns it via
    :meth:`get_action` for teleoperation or recording.

    Calibration is reused from the local ``so_follower`` directory (same as
    :class:`RemoteSO101`) so that Leader and Follower share the same coordinate
    system.
    """

    config_class = RemoteSO101LeaderConfig
    name = "remote_so101_leader"

    JOINTS = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]

    def __init__(self, config: RemoteSO101LeaderConfig):
        if config.calibration_dir is None:
            config.calibration_dir = HF_LEROBOT_CALIBRATION / TELEOPERATORS / "so_leader"
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._obs_sock: socket.socket | None = None
        self._recv_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._latest_action: RobotAction = {}
        self._lock = threading.Lock()
        self._setup_signal_handlers()

    def _setup_signal_handlers(self) -> None:
        def _signal_handler(signum, frame):
            logger.info(f"Received signal {signum}, disconnecting...")
            try:
                self.disconnect()
            except Exception:
                pass
            sys.exit(0)

        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in self.JOINTS}

    @property
    def feedback_features(self) -> dict[str, type]:
        # No force feedback yet
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return bool(self.calibration)

    def calibrate(self) -> None:
        """Remote leader does not support interactive calibration."""
        raise NotImplementedError(
            "RemoteSO101Leader does not support calibration. "
            "Please calibrate the local SO101 robot first and copy the calibration file, "
            "or specify the correct calibration_dir pointing to the existing SO101 calibration."
        )

    def configure(self) -> None:
        """No-op for remote teleoperator."""
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        if not self.is_calibrated and calibrate:
            raise RuntimeError(
                "No calibration file found for remote SO101 Leader. "
                f"Expected: {self.calibration_fpath}. "
                "Please ensure the local SO101 calibration file exists at that path, "
                "or pass --teleop.calibration_dir pointing to the correct directory."
            )

        self._obs_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._obs_sock.settimeout(self.config.timeout_s)
        self._obs_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._obs_sock.connect((self.config.remote_ip, self.config.port_obs))

        self._is_connected = True
        self._stop_event.clear()
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        logger.info(f"RemoteSO101Leader connected to {self.config.remote_ip}")

    def _recv_loop(self) -> None:
        """Background thread that continuously reads streamed positions from ESP32."""
        while not self._stop_event.is_set() and self._is_connected:
            try:
                data = self._recv_obs()
                if data is not None:
                    action = self._normalize_observation(data)
                    with self._lock:
                        self._latest_action = action
            except Exception as exc:
                logger.debug(f"Recv loop error: {exc}")
                time.sleep(0.01)

    def _recv_obs(self) -> dict | None:
        """Receive one JSON observation line from ESP32."""
        if not self._obs_sock or not self._is_connected:
            return None
        try:
            buf = b""
            while b"\n" not in buf:
                chunk = self._obs_sock.recv(4096)
                if not chunk:
                    logger.warning("Observation socket closed by peer.")
                    self._is_connected = False
                    return None
                buf += chunk
            return json.loads(buf.decode().strip())
        except socket.timeout:
            logger.debug("Observation read timeout")
            return None
        except (ConnectionResetError, OSError) as exc:
            logger.warning(f"Observation recv failed: {exc}")
            self._is_connected = False
            return None

    def _normalize_observation(self, data: dict) -> RobotAction:
        """Convert native motor steps (0-4095) to normalized action values.

        Mirrors the normalisation logic in
        :meth:`lerobot.motors.SerialMotorsBus._normalize` for the
        ``RANGE_M100_100`` mode.
        """
        action: RobotAction = {}
        for joint in self.JOINTS:
            key = f"{joint}.pos"
            native_val = int(data.get(joint, 2048))
            cal = self.calibration[joint]
            min_ = int(cal.range_min)
            max_ = int(cal.range_max)
            drive_mode = int(cal.drive_mode)

            if self.config.use_degrees:
                mid = (min_ + max_) / 2.0
                max_res = 4095
                val_deg = (native_val - mid) * 360.0 / max_res
                action[key] = float(val_deg)
            else:
                bounded = float(np.clip(native_val, min_, max_))
                norm = 200.0 * (bounded - min_) / (max_ - min_) - 100.0
                action[key] = -norm if drive_mode else norm
        return action

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        """Return the latest action from the Leader arm.

        Because the background thread continuously receives streamed positions,
        this method is non-blocking and simply returns the most recent frame.
        """
        with self._lock:
            return self._latest_action.copy() if self._latest_action else {}

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO: Implement force feedback
        raise NotImplementedError

    @check_if_not_connected
    def disconnect(self) -> None:
        self._stop_event.set()
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)
        if self._obs_sock:
            try:
                self._obs_sock.close()
            except Exception:
                pass
        self._is_connected = False
        logger.info("RemoteSO101Leader disconnected.")
