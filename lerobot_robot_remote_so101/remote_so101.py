import json
import logging
import socket

import numpy as np
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config import RemoteSO101Config

logger = logging.getLogger(__name__)


class RemoteSO101(Robot):
    """Remote SO101 robot implementation that delegates control to an ESP32 over TCP.

    This robot reuses SO101 calibration files (stored under the ``so_follower``
    directory) to perform the exact same denormalisation that the local
    :class:`~lerobot.robots.so_follower.SOFollower` applies before writing to
    the Feetech motors.
    """

    config_class = RemoteSO101Config
    name = "remote_so101"

    # Joint names must match the action feature names used by SO101 datasets.
    JOINTS = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]

    def __init__(self, config: RemoteSO101Config):
        # Reuse SO101/SO100 calibration directory by default so the user does
        # not have to duplicate calibration files.
        if config.calibration_dir is None:
            config.calibration_dir = HF_LEROBOT_CALIBRATION / ROBOTS / "so_follower"
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._cmd_sock: socket.socket | None = None
        self._obs_sock: socket.socket | None = None
        self._last_obs: RobotObservation = {}

    @property
    def observation_features(self) -> dict[str, type]:
        return {f"{j}.pos": float for j in self.JOINTS}

    @property
    def action_features(self) -> dict[str, type]:
        return self.observation_features

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        # Calibration is managed on the PC side (reuses SO101 files).
        return bool(self.calibration)

    def calibrate(self) -> None:
        """Remote robot does not support interactive calibration."""
        raise NotImplementedError(
            "RemoteSO101 does not support calibration. "
            "Please calibrate the local SO101 robot first and copy the calibration file, "
            "or specify the correct calibration_dir pointing to the existing SO101 calibration."
        )

    def configure(self) -> None:
        """No-op for remote robot."""
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        if not self.is_calibrated and calibrate:
            raise RuntimeError(
                "No calibration file found for remote SO101. "
                f"Expected: {self.calibration_fpath}. "
                "Please ensure the local SO101 calibration file exists at that path, "
                "or pass --robot.calibration_dir pointing to the correct directory."
            )

        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._cmd_sock.settimeout(self.config.timeout_s)
        self._cmd_sock.connect((self.config.remote_ip, self.config.port_cmd))

        self._obs_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._obs_sock.settimeout(self.config.timeout_s)
        self._obs_sock.connect((self.config.remote_ip, self.config.port_obs))

        self._is_connected = True
        logger.info(f"RemoteSO101 connected to {self.config.remote_ip}")

    def _denormalize(self, action: RobotAction) -> dict[str, int]:
        """Convert normalised action values back to native motor steps (0-4095).

        This mirrors the denormalisation logic inside
        :meth:`lerobot.motors.SerialMotorsBus._unnormalize` for the
        ``RANGE_M100_100`` mode (default for SO101 when ``use_degrees=False``).
        """
        if not self.calibration:
            raise RuntimeError(
                "Cannot denormalise actions without calibration data. "
                "Ensure calibration file is present and loaded."
            )

        native: dict[str, int] = {}
        for joint in self.JOINTS:
            key = f"{joint}.pos"
            norm_val = float(action[key])
            cal = self.calibration[joint]
            min_ = int(cal.range_min)
            max_ = int(cal.range_max)
            drive_mode = int(cal.drive_mode)

            if self.config.use_degrees:
                # Degrees mode: mid = (min+max)/2, resolution = 4095
                mid = (min_ + max_) / 2.0
                max_res = 4095
                val_deg = float(norm_val)
                native_val = int((val_deg * max_res / 360.0) + mid)
            else:
                # RANGE_M100_100 mode: norm_val in [-100, 100]
                val = -norm_val if drive_mode else norm_val
                bounded = min(100.0, max(-100.0, val))
                native_val = int(((bounded + 100.0) / 200.0) * (max_ - min_) + min_)

            native[joint] = int(np.clip(native_val, min_, max_))
        return native

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        native_positions = self._denormalize(action)
        payload = {
            "cmd": "set_positions",
            "positions": native_positions,
        }
        msg = json.dumps(payload, separators=(",", ":")) + "\n"
        self._cmd_sock.sendall(msg.encode())
        return action

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        try:
            request = json.dumps({"cmd": "get_obs"}, separators=(",", ":")) + "\n"
            self._cmd_sock.sendall(request.encode())

            buf = b""
            while b"\n" not in buf:
                chunk = self._obs_sock.recv(4096)
                if not chunk:
                    break
                buf += chunk

            data = json.loads(buf.decode().strip())
            self._last_obs = {f"{k}.pos": float(v) for k, v in data.items()}
        except Exception as exc:
            logger.warning(f"Observation read failed: {exc}; reusing last known values.")
        return self._last_obs

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._cmd_sock:
            self._cmd_sock.close()
        if self._obs_sock:
            self._obs_sock.close()
        self._is_connected = False
        logger.info("RemoteSO101 disconnected.")
