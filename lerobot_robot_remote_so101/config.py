from dataclasses import dataclass, field
from pathlib import Path
from lerobot.robots.config import RobotConfig
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

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Override default calibration directory to reuse SO101 calibration files.
    # SO101 and SO100 share the same "so_follower" robot name in LeRobot,
    # so their calibration files live under the "so_follower" directory.
    calibration_dir: Path | None = field(default=None, repr=False)
