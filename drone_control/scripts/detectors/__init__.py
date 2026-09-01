# drone_control/scripts/detectors/__init__.py
# Detectors package initialization

from .object_tracker import ObjectTracker
from .yolo_detector import YOLODetector

__all__ = [
    'ObjectTracker',
     'YOLODetector'
]