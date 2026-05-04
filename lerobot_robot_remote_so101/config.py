from dataclasses import dataclass, field
from pathlib import Path
from lerobot.robots.config import RobotConfig
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.cameras import CameraConfig


@RobotConfig.register_subclass("remote_so101")
@dataclass
class RemoteSO101Config(RobotConfig):
    """Configuration for remote SO101 robot controlled via ESP32 TCP bridge.

    The calibration files are shared with the local SO101/SO100 follower,
    since the joint structure and motor models are identical.
    """

    remote_ip: str = "192.168.4.1"
    port_cmd: int = 8888
    port_obs: int = 8889
    timeout_s: float = 5.0

    use_degrees: bool = False

    # Skip observation reads during replay. This eliminates the ~20-50ms RTT
    # penalty per frame and restores smooth 30fps replay. Safe for replay since
    # actions are pre-computed in the dataset. Do NOT enable for teleoperation
    # or closed-loop control where real-time feedback is required.
    skip_observation: bool = False

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Override default calibration directory to reuse SO101 calibration files.
    # SO101 and SO100 share the same "so_follower" robot name in LeRobot,
    # so their calibration files live under the "so_follower" directory.
    calibration_dir: Path | None = field(default=None, repr=False)


@TeleoperatorConfig.register_subclass("remote_so101_leader")
@dataclass(kw_only=True)
class RemoteSO101LeaderConfig(TeleoperatorConfig):
    """Configuration for remote SO101 Leader teleoperator controlled via ESP32 TCP bridge.

    The Leader ESP32 streams joint positions at 30 Hz. This teleoperator
    receives the stream and returns actions for teleoperation or recording.

    Calibration is reused from the local SO101/SO100 follower.
    """

    remote_ip: str = "192.168.4.2"
    port_obs: int = 8889
    timeout_s: float = 5.0

    use_degrees: bool = False
