#!/usr/bin/env python3
"""
Stereo Depth Estimation for CLOVER Rover
Uses OpenCV StereoSGBM for disparity calculation
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
from loguru import logger


@dataclass
class StereoCalibration:
    """Stereo camera calibration parameters"""
    baseline: float = 0.06  # 6cm baseline between cameras (measure your setup!)
    focal_length: float = 500.0  # Focal length in pixels (calibrate for your camera)
    cx: float = 640.0  # Principal point x
    cy: float = 360.0  # Principal point y


class StereoDepthEstimator:
    """
    Stereo depth estimation using Semi-Global Block Matching (SGBM)
    Optimized for indoor robotics applications
    """

    def __init__(self, calibration: Optional[StereoCalibration] = None):
        """
        Initialize stereo depth estimator

        Args:
            calibration: Camera calibration parameters
        """
        self.calibration = calibration or StereoCalibration()

        # SGBM parameters tuned for indoor environments
        self.min_disparity = 0
        self.num_disparities = 128  # Must be divisible by 16
        self.block_size = 5  # Odd number, 3-11 recommended
        self.p1 = 8 * 3 * self.block_size ** 2
        self.p2 = 32 * 3 * self.block_size ** 2
        self.disp12_max_diff = 1
        self.uniqueness_ratio = 10
        self.speckle_window_size = 100
        self.speckle_range = 32
        self.pre_filter_cap = 63

        self._create_matcher()
        logger.info(f"StereoDepthEstimator initialized (baseline={self.calibration.baseline}m)")

    def _create_matcher(self):
        """Create SGBM matcher"""
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=self.min_disparity,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=self.p1,
            P2=self.p2,
            disp12MaxDiff=self.disp12_max_diff,
            uniquenessRatio=self.uniqueness_ratio,
            speckleWindowSize=self.speckle_window_size,
            speckleRange=self.speckle_range,
            preFilterCap=self.pre_filter_cap,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # WLS filter for better results (requires opencv-contrib)
        self.wls_available = hasattr(cv2, 'ximgproc')
        if self.wls_available:
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
            self.wls_filter.setLambda(8000)
            self.wls_filter.setSigmaColor(1.5)
            logger.info("WLS filter enabled")
        else:
            self.right_matcher = None
            self.wls_filter = None
            logger.warning("WLS filter not available (opencv-contrib not installed)")

    def compute_disparity(self, left: np.ndarray, right: np.ndarray,
                         use_wls: bool = True) -> np.ndarray:
        """
        Compute disparity map from stereo pair

        Args:
            left: Left image (BGR or grayscale)
            right: Right image (BGR or grayscale)
            use_wls: Use WLS filter for smoother results

        Returns:
            Disparity map (float32)
        """
        # Convert to grayscale if needed
        if len(left.shape) == 3:
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        else:
            left_gray = left
            right_gray = right

        # Compute disparity
        left_disp = self.stereo.compute(left_gray, right_gray)

        if use_wls and self.wls_available:
            right_disp = self.right_matcher.compute(right_gray, left_gray)
            disparity = self.wls_filter.filter(left_disp, left_gray, None, right_disp)
        else:
            disparity = left_disp

        # Convert to float and normalize
        disparity = disparity.astype(np.float32) / 16.0

        return disparity

    def disparity_to_depth(self, disparity: np.ndarray) -> np.ndarray:
        """
        Convert disparity to depth in meters

        Args:
            disparity: Disparity map

        Returns:
            Depth map in meters
        """
        # Avoid division by zero
        disparity_safe = np.where(disparity > 0, disparity, 0.1)

        # depth = baseline * focal_length / disparity
        depth = (self.calibration.baseline * self.calibration.focal_length) / disparity_safe

        # Clamp to reasonable range (0.1m to 10m)
        depth = np.clip(depth, 0.1, 10.0)

        # Set invalid disparities to max distance
        depth[disparity <= 0] = 10.0

        return depth

    def compute_depth(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """
        Compute depth map from stereo pair

        Args:
            left: Left image
            right: Right image

        Returns:
            Depth map in meters
        """
        disparity = self.compute_disparity(left, right)
        return self.disparity_to_depth(disparity)

    def get_depth_at_point(self, depth_map: np.ndarray, x: int, y: int,
                          window_size: int = 5) -> float:
        """
        Get depth at a specific point with averaging

        Args:
            depth_map: Depth map in meters
            x, y: Point coordinates
            window_size: Averaging window size

        Returns:
            Depth in meters
        """
        h, w = depth_map.shape[:2]
        half = window_size // 2

        # Clamp to valid range
        x1 = max(0, x - half)
        x2 = min(w, x + half + 1)
        y1 = max(0, y - half)
        y2 = min(h, y + half + 1)

        # Get median depth in window (more robust than mean)
        window = depth_map[y1:y2, x1:x2]
        valid = window[window < 10.0]

        if len(valid) > 0:
            return float(np.median(valid))
        return 10.0  # Max distance if no valid depth

    def get_depth_in_bbox(self, depth_map: np.ndarray,
                         bbox: Tuple[int, int, int, int]) -> float:
        """
        Get average depth within a bounding box

        Args:
            depth_map: Depth map in meters
            bbox: Bounding box (x1, y1, x2, y2)

        Returns:
            Median depth in meters
        """
        x1, y1, x2, y2 = bbox
        h, w = depth_map.shape[:2]

        # Clamp to valid range
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)

        if x2 <= x1 or y2 <= y1:
            return 10.0

        # Use center region (more reliable)
        cx1 = x1 + (x2 - x1) // 4
        cx2 = x2 - (x2 - x1) // 4
        cy1 = y1 + (y2 - y1) // 4
        cy2 = y2 - (y2 - y1) // 4

        region = depth_map[cy1:cy2, cx1:cx2]
        valid = region[region < 10.0]

        if len(valid) > 0:
            return float(np.median(valid))
        return 10.0

    def colorize_depth(self, depth_map: np.ndarray,
                      min_depth: float = 0.2,
                      max_depth: float = 3.0) -> np.ndarray:
        """
        Create colorized visualization of depth map

        Args:
            depth_map: Depth map in meters
            min_depth: Minimum depth for colormap
            max_depth: Maximum depth for colormap

        Returns:
            Colorized depth image (BGR)
        """
        # Normalize to 0-255
        depth_normalized = (depth_map - min_depth) / (max_depth - min_depth)
        depth_normalized = np.clip(depth_normalized, 0, 1)
        depth_uint8 = (depth_normalized * 255).astype(np.uint8)

        # Apply colormap (TURBO: red=close, blue=far)
        colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_TURBO)

        return colored


if __name__ == "__main__":
    logger.info("Stereo depth module loaded")
    logger.info("Test requires stereo camera images")
