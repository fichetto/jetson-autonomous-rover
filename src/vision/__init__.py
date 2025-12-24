"""
Computer vision module with CUDA/TensorRT acceleration
"""

from .camera import JetsonCamera
from .detector import ObjectDetector, Detection

__all__ = ['JetsonCamera', 'ObjectDetector', 'Detection']
