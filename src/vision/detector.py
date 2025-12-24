#!/usr/bin/env python3
"""
Object Detection with CUDA/TensorRT acceleration
Optimized for Jetson Orin Nano
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from loguru import logger

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    logger.warning("ONNXRuntime not available")


@dataclass
class Detection:
    """Single object detection"""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2


class ObjectDetector:
    """
    Object detector using ONNX/TensorRT
    Compatible with YOLOv8, YOLOv5, and similar models
    """

    def __init__(self, model_path: str, conf_threshold: float = 0.5,
                 nms_threshold: float = 0.4, input_size: Tuple[int, int] = (640, 640)):
        """
        Initialize object detector

        Args:
            model_path: Path to ONNX model file
            conf_threshold: Confidence threshold
            nms_threshold: NMS threshold
            input_size: Model input size (width, height)
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.input_size = input_size

        self.session: Optional[ort.InferenceSession] = None
        self.input_name: Optional[str] = None
        self.output_names: Optional[List[str]] = None

        # COCO class names (default)
        self.class_names = self._get_coco_names()

        logger.info(f"Initializing detector with model: {model_path}")

    def load_model(self) -> bool:
        """
        Load ONNX model with CUDA/TensorRT optimization

        Returns:
            True if successful
        """
        if not ONNX_AVAILABLE:
            logger.error("ONNXRuntime not installed")
            return False

        try:
            # Configure CUDA Execution Provider for Jetson
            providers = [
                ('CUDAExecutionProvider', {
                    'device_id': 0,
                    'arena_extend_strategy': 'kNextPowerOfTwo',
                    'gpu_mem_limit': 2 * 1024 * 1024 * 1024,  # 2GB
                    'cudnn_conv_algo_search': 'EXHAUSTIVE',
                    'do_copy_in_default_stream': True,
                }),
                'CPUExecutionProvider',  # Fallback
            ]

            # Try TensorRT if available
            try:
                providers.insert(0, ('TensorrtExecutionProvider', {
                    'device_id': 0,
                    'trt_max_workspace_size': 2147483648,  # 2GB
                    'trt_fp16_enable': True,
                }))
                logger.info("TensorRT provider available")
            except Exception:
                logger.info("TensorRT provider not available, using CUDA")

            self.session = ort.InferenceSession(self.model_path, providers=providers)

            # Get input/output names
            self.input_name = self.session.get_inputs()[0].name
            self.output_names = [output.name for output in self.session.get_outputs()]

            # Log provider info
            provider = self.session.get_providers()[0]
            logger.success(f"Model loaded with {provider}")

            return True

        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            return False

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image for inference

        Args:
            image: Input BGR image

        Returns:
            Preprocessed image tensor
        """
        # Resize
        input_img = cv2.resize(image, self.input_size)

        # Convert BGR to RGB
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)

        # Normalize to [0, 1]
        input_img = input_img.astype(np.float32) / 255.0

        # HWC to CHW
        input_img = np.transpose(input_img, (2, 0, 1))

        # Add batch dimension
        input_img = np.expand_dims(input_img, axis=0)

        return input_img

    def postprocess(self, outputs: np.ndarray, orig_shape: Tuple[int, int]
                   ) -> List[Detection]:
        """
        Postprocess model outputs

        Args:
            outputs: Raw model outputs
            orig_shape: Original image shape (height, width)

        Returns:
            List of detections
        """
        detections = []

        # Handle different output formats
        if len(outputs.shape) == 3:
            outputs = outputs[0]  # Remove batch dimension

        # YOLOv8 format: (num_boxes, 4 + num_classes)
        # First 4 values are [x_center, y_center, width, height]
        # Remaining are class scores

        num_classes = outputs.shape[1] - 4

        for detection in outputs:
            scores = detection[4:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence < self.conf_threshold:
                continue

            # Extract bbox
            x_center, y_center, width, height = detection[:4]

            # Scale to original image
            scale_x = orig_shape[1] / self.input_size[0]
            scale_y = orig_shape[0] / self.input_size[1]

            x_center *= scale_x
            y_center *= scale_y
            width *= scale_x
            height *= scale_y

            # Convert to x1, y1, x2, y2
            x1 = int(x_center - width / 2)
            y1 = int(y_center - height / 2)
            x2 = int(x_center + width / 2)
            y2 = int(y_center + height / 2)

            detections.append(Detection(
                class_id=int(class_id),
                class_name=self.class_names.get(int(class_id), f"class_{class_id}"),
                confidence=float(confidence),
                bbox=(x1, y1, x2, y2)
            ))

        # Apply NMS
        detections = self._non_max_suppression(detections)

        return detections

    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        Run object detection on image

        Args:
            image: Input BGR image

        Returns:
            List of detections
        """
        if self.session is None:
            logger.error("Model not loaded")
            return []

        try:
            # Preprocess
            input_tensor = self.preprocess(image)

            # Inference
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})

            # Postprocess
            detections = self.postprocess(outputs[0], image.shape[:2])

            return detections

        except Exception as e:
            logger.error(f"Detection error: {e}")
            return []

    def _non_max_suppression(self, detections: List[Detection]) -> List[Detection]:
        """
        Apply Non-Maximum Suppression

        Args:
            detections: List of detections

        Returns:
            Filtered detections
        """
        if not detections:
            return []

        # Convert to numpy arrays for OpenCV NMS
        boxes = np.array([det.bbox for det in detections])
        scores = np.array([det.confidence for det in detections])

        # OpenCV NMS expects [x1, y1, width, height]
        boxes_xywh = boxes.copy()
        boxes_xywh[:, 2] = boxes[:, 2] - boxes[:, 0]  # width
        boxes_xywh[:, 3] = boxes[:, 3] - boxes[:, 1]  # height

        indices = cv2.dnn.NMSBoxes(
            boxes_xywh.tolist(),
            scores.tolist(),
            self.conf_threshold,
            self.nms_threshold
        )

        if len(indices) > 0:
            return [detections[i] for i in indices.flatten()]

        return []

    def draw_detections(self, image: np.ndarray, detections: List[Detection]
                       ) -> np.ndarray:
        """
        Draw detection results on image

        Args:
            image: Input image
            detections: List of detections

        Returns:
            Image with drawn detections
        """
        result = image.copy()

        for det in detections:
            x1, y1, x2, y2 = det.bbox

            # Draw box
            cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label
            label = f"{det.class_name}: {det.confidence:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

            cv2.rectangle(result, (x1, y1 - label_h - 10), (x1 + label_w, y1), (0, 255, 0), -1)
            cv2.putText(result, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return result

    @staticmethod
    def _get_coco_names() -> Dict[int, str]:
        """Get COCO class names"""
        return {
            0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
            5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
            10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
            14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
            20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
            25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
            # ... add more as needed
        }


if __name__ == "__main__":
    # Test detector (requires model file)
    logger.info("Detector module loaded")
    logger.info("Note: Actual testing requires ONNX model file")
