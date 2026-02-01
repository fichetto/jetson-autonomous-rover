"""
Computer vision module with CUDA/TensorRT acceleration
"""

from .camera import JetsonCamera
from .detector import ObjectDetector, Detection
from .stereo_camera import StereoCamera, StereoFrame
from .stereo_depth import StereoDepthEstimator, StereoCalibration
from .cat_follower import CatFollower, CatDetector

__all__ = [
    'JetsonCamera',
    'ObjectDetector',
    'Detection',
    'StereoCamera',
    'StereoFrame',
    'StereoDepthEstimator',
    'StereoCalibration',
    'CatFollower',
    'CatDetector',
]
