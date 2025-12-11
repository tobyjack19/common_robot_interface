from .base import GripperControllerBase
from .crap_hand_controller import CRAPHandController

# Public mapping to construct grippers similarly to robot Controller map
Gripper = {
    'crap_hand': CRAPHandController,
}

__all__ = [
    'GripperControllerBase',
    'CRAPHandController',
    'Gripper',
]
