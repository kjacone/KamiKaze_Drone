# drone_control/scripts/detectors/__init__.py
# Detectors package initialization

from .yolo_detector import YOLODetector
from .object_tracker import ObjectTracker

__all__ = [
    'YOLODetector',
    'ObjectTracker'
]