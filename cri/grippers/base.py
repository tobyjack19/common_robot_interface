from abc import ABC, abstractmethod
from typing import Optional, Dict, Any


class GripperControllerBase(ABC):
    """Abstract base class for gripper controllers.

    A gripper exposes basic primitives like enabling power, homing,
    setting a target open/close position, and optionally speed/accel/limits.

    Concrete implementations should translate these calls to the specific
    hardware API and handle any required connection lifecycle.
    """

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    @property
    @abstractmethod
    def info(self) -> str:
        """Human-readable identifier for this gripper instance."""
        raise NotImplementedError

    # Capability queries (optional to override)
    def has_limits(self) -> bool:
        return False

    # State queries (optional to override)
    def current_position(self) -> Optional[float]:
        """Return current open fraction in [0,1] if available."""
        return None

    def target_position(self) -> Optional[float]:
        """Return target open fraction in [0,1] if available."""
        return None

    # Control primitives
    @abstractmethod
    def enable(self, on: bool) -> None:
        """Enable or disable gripper power/driver."""
        raise NotImplementedError

    def home(self) -> None:
        """Home the gripper if supported."""
        pass

    def home_index(self) -> None:
        """Home using index sensor if supported."""
        pass

    def set_speed(self, value: float) -> None:
        """Set motion speed (units are gripper-specific)."""
        pass

    def set_accel(self, value: float) -> None:
        """Set motion acceleration (units are gripper-specific)."""
        pass

    def set_limits(self, min_pos: int, max_pos: int) -> None:
        """Optionally set hardware position limits if supported."""
        pass

    @abstractmethod
    def set_position(self, value_01: float, wait: bool = False, timeout_s: float = 30.0) -> None:
        """Command target open fraction in [0,1]."""
        raise NotImplementedError

    def jog(self, delta_steps: int) -> None:
        """Jog by relative steps if supported."""
        pass

    def status(self) -> Dict[str, Any]:
        """Return driver-specific status fields (best-effort)."""
        return {}

    @abstractmethod
    def close(self) -> None:
        """Release transport/resources."""
        raise NotImplementedError
