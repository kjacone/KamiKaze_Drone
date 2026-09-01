#!/usr/bin/env python3
"""
drone_control/scripts/detectors/yolo_detector.py
Enhanced YOLO detector with structured logging and metrics
"""

import os
import sys
import time
import traceback

import cv2
import numpy as np
import psutil
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

from drone_control.utils import ErrorHandler, MessageValidator
from drone_control.utils.correlation import get_or_create_mission_id
from drone_control.utils.logging_framework import get_logger_with_ros_level
from drone_control.utils.metrics_collector import MetricsCollector, MetricTimer
from drone_control.msg import (
    BBox,
    Command,
    CommandResponse,
    DetectedObjects,
    MissionStatus,
    NodeHealth,
    Object,
)



class YOLODetector:
    """Enhanced YOLO detector with full system integration, structured logging, and metrics"""

    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=False)

        # Initialize structured logger
        self.logger = get_logger_with_ros_level("yolo_detector")
        self.logger.info("node_initializing", extra={
            "version": "1.0.0",
            "simulation_mode": rospy.get_param('/use_simulation', True)
        })

        # Setup components
        self.bridge = CvBridge()
        self.error_handler = ErrorHandler(node_name='yolo_detector')
        self.message_validator = MessageValidator()

        # Get parameters (must happen before _init_metrics(), which reads
        # self.model_path / self.device to populate the model_loaded gauge)
        self.model_path = rospy.get_param('~model_path', 'yolov8s.pt')
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.4)
        self.input_size = rospy.get_param('~input_size', 416)
        self.device = rospy.get_param('~device', 'cpu')
        self.vehicle_classes = rospy.get_param('detection/vehicle_classes', [2, 3, 5, 6, 7])

        # Mode flags
        self.debug_mode = rospy.get_param('~debug_mode', False)
        self.test_mode = rospy.get_param('~test_mode', False)
        self.simulation_mode = rospy.get_param('/use_simulation', True)

        # Test mode parameters
        if self.test_mode:
            self.mock_accuracy = rospy.get_param('~mock_accuracy', 0.95)
            self.detection_latency = rospy.get_param('~detection_latency', 0.05)
            self.false_positive_rate = rospy.get_param('~false_positive_rate', 0.02)

        # Initialize metrics collector (now safe: model_path/device already set)
        self.metrics = MetricsCollector("yolo_detector", port=8001)
        self._init_metrics()

        # State variables
        self.node_health = "running"
        self.last_health_publish = time.time()
        self.detection_count = 0
        self.error_count = 0
        self.fps = 0.0
        self.frame_times = []
        self.current_mission_id = get_or_create_mission_id()
        self.last_detection_time = time.time()
        self.total_detection_time = 0.0
        self.detection_attempts = 0
        self.false_positives = 0

        # Health monitoring
        self.health_pub = rospy.Publisher('/yolo_detector/node_health', NodeHealth, queue_size=10)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)

        # Command handling
        self.command_sub = rospy.Subscriber('/drone_control/command', Command, self._handle_command)
        self.response_pub = rospy.Publisher('/command_response', CommandResponse, queue_size=10)

        # Mission integration
        self.mission_sub = rospy.Subscriber('/mission_status', MissionStatus, self._handle_mission_status)
        self.current_mission_state = 'idle'

        # Load model
        self.model = self._load_model()

        # Subscribers and Publishers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.objects_pub = rospy.Publisher('/detected_objects', DetectedObjects, queue_size=10)

        # Performance monitoring
        self.perf_timer = rospy.Timer(rospy.Duration(5.0), self._publish_performance)

        self.logger.info("node_initialized", extra={
            "model_path": self.model_path,
            "device": self.device,
            "test_mode": self.test_mode,
            "simulation_mode": self.simulation_mode
        })

    def _init_metrics(self):
        """Initialize Prometheus metrics for this node"""
        self.logger.debug("initializing_metrics")
        
        # Counters
        self.metric_detections_total = self.metrics.counter(
            "detections_total",
            "Total number of detections",
            labels=["class_id", "class_name", "mission_id"]
        )
        self.metric_frames_processed_total = self.metrics.counter(
            "frames_processed_total",
            "Total number of frames processed",
            labels=["mission_id"]
        )
        self.metric_errors_total = self.metrics.counter(
            "errors_total",
            "Total number of errors",
            labels=["error_type", "mission_id"]
        )
        self.metric_false_positives_total = self.metrics.counter(
            "false_positives_total",
            "Total number of false positive detections",
            labels=["mission_id"]
        )
        
        # Gauges
        self.metric_current_fps = self.metrics.gauge(
            "fps",
            "Current frames per second",
            labels=["mission_id"]
        )
        self.metric_model_loaded = self.metrics.gauge(
            "model_loaded",
            "Whether model is loaded (1=loaded, 0=not)",
            labels=["model_path", "device"]
        )
        self.metric_health_status = self.metrics.gauge(
            "health_status",
            "Health status (1=healthy, 0=unhealthy)",
            labels=["mission_id"]
        )
        
        # Histograms
        self.metric_detection_latency = self.metrics.histogram(
            "detection_latency_seconds",
            "Detection processing time in seconds",
            labels=["mission_id"],
            buckets=[0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0]
        )
        self.metric_confidence_histogram = self.metrics.histogram(
            "confidence_histogram",
            "Distribution of detection confidences",
            labels=["class_id", "mission_id"],
            buckets=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
        )
        
        # Set model loaded gauge
        self.metric_model_loaded.labels(
            model_path=self.model_path,
            device=self.device
        ).set(1)
        
        self.logger.info("metrics_initialized")

    def _load_model(self):
        """Load YOLO model with validation"""
        try:
            self.logger.info("loading_model", extra={
                "model_path": self.model_path,
                "device": self.device
            })

            # Check if model exists
            if not os.path.exists(self.model_path) and self.model_path != 'yolov8s.pt':
                raise FileNotFoundError(f"Model not found: {self.model_path}")

            # Load model
            model = YOLO(self.model_path)

            # Validate model loaded
            if model is None:
                raise RuntimeError("Failed to load YOLO model")

            # Move model onto the configured device
            try:
                model.to(self.device)
            except Exception as device_err:
                self.logger.warning("device_move_failed", extra={
                    "device": self.device,
                    "error": str(device_err)
                })

            self.logger.info("model_loaded_successfully")
            return model

        except Exception as e:
            self.logger.exception("model_loading_failed", extra={
                "model_path": self.model_path,
                "error": str(e)
            })
            self.error_handler.handle_error(e, "Model loading failed")
            rospy.signal_shutdown(f"Model loading failed: {e}")
            sys.exit(1)

    def _handle_command(self, msg):
        """Handle incoming commands"""
        self.logger.debug("command_received", extra={
            "command": msg.command,
            "parameters": getattr(msg, 'parameters', {})
        })

        try:
            if msg.command == 'reload_model':
                self._reload_model()
                self._send_response(msg.command, True, "Model reloaded")
            elif msg.command == 'set_confidence':
                self.conf_threshold = msg.parameters.get('threshold', 0.5)
                self.logger.info("confidence_threshold_updated", extra={
                    "new_threshold": self.conf_threshold
                })
                self._send_response(msg.command, True, f"Confidence set to {self.conf_threshold}")
            elif msg.command == 'enable_debug':
                self.debug_mode = True
                self.logger.info("debug_mode_enabled")
                self._send_response(msg.command, True, "Debug mode enabled")
            elif msg.command == 'disable_debug':
                self.debug_mode = False
                self.logger.info("debug_mode_disabled")
                self._send_response(msg.command, True, "Debug mode disabled")
            else:
                self.logger.warning("unknown_command", extra={
                    "command": msg.command
                })
                self._send_response(msg.command, False, f"Unknown command: {msg.command}")

        except Exception as e:
            self.logger.exception("command_handling_failed", extra={
                "command": msg.command,
                "error": str(e)
            })
            self.error_handler.handle_error(e, f"Command: {msg.command}")
            self._send_response(msg.command, False, str(e))

    def _send_response(self, command, success, message):
        """Send command response"""
        response = CommandResponse()
        response.original_command = command
        response.success = success
        response.message = message
        self.response_pub.publish(response)
        self.logger.debug("command_response_sent", extra={
            "command": command,
            "success": success,
            "message": message
        })

    def _handle_mission_status(self, msg):
        """Handle mission status updates"""
        self.current_mission_state = msg.state
        self.current_mission_id = msg.mission_id
        if self.debug_mode:
            self.logger.debug("mission_status_updated", extra={
                "state": msg.state,
                "mission_id": msg.mission_id
            })

    def _reload_model(self):
        """Reload the model at runtime"""
        try:
            self.logger.info("reloading_model")
            model = YOLO(self.model_path)
            try:
                model.to(self.device)
            except Exception as device_err:
                self.logger.warning("device_move_failed_on_reload", extra={
                    "device": self.device,
                    "error": str(device_err)
                })
            self.model = model
            self.logger.info("model_reloaded_successfully")
        except Exception as e:
            self.logger.exception("model_reload_failed", extra={
                "error": str(e)
            })
            self.error_handler.handle_error(e, "Model reload failed")
            raise

    def _validate_image(self, msg):
        """Validate incoming image message"""
        is_valid, error = self.message_validator.validate_image(msg)
        if not is_valid:
            self.logger.warning("invalid_image_message", extra={
                "error": error
            })
        return is_valid, error

    def _publish_performance(self, event):
        """Publish performance metrics"""
        if self.frame_times:
            total_time = sum(self.frame_times)
            avg_fps = len(self.frame_times) / total_time if total_time > 0 else 0
            self.fps = avg_fps
            
            # Update FPS gauge
            self.metric_current_fps.labels(
                mission_id=self.current_mission_id
            ).set(avg_fps)
            
            self.logger.debug("performance_metrics", extra={
                "fps": avg_fps,
                "detections": self.detection_count,
                "frames_processed": len(self.frame_times)
            })
            self.frame_times = []

    def image_callback(self, msg):
        """Process incoming image"""
        start_time = time.time()
        self.detection_attempts += 1

        # Log frame start with image metadata
        self.logger.debug("frame_processing_started", extra={
            "frame_id": self.detection_attempts,
            "image_size": [msg.width, msg.height],
            "encoding": msg.encoding
        })

        try:
            # Validate message
            is_valid, error = self._validate_image(msg)
            if not is_valid:
                self.error_count += 1
                self.metric_errors_total.labels(
                    error_type="invalid_image",
                    mission_id=self.current_mission_id
                ).inc()
                self.logger.warning("frame_skipped_invalid_image", extra={
                    "error": error,
                    "frame_id": self.detection_attempts
                })
                return

            # Convert ROS image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.logger.debug("image_converted", extra={
                "frame_id": self.detection_attempts,
                "shape": cv_img.shape
            })

            # Test mode - simulate detections
            if self.test_mode:
                detections = self._simulate_detections(cv_img)
            else:
                # Run inference with timing
                with MetricTimer(self.metric_detection_latency, labels={"mission_id": self.current_mission_id}):
                    detections = self._run_inference(cv_img)

            # Publish detections
            if detections is not None and len(detections.objects) > 0:
                self.objects_pub.publish(detections)
                self.detection_count += len(detections.objects)
                
                # Log detection results
                self.logger.info("objects_detected", extra={
                    "frame_id": self.detection_attempts,
                    "count": len(detections.objects),
                    "classes": [obj.class_name for obj in detections.objects],
                    "confidences": [obj.confidence for obj in detections.objects]
                })
                
                # Update detection counter metrics
                for obj in detections.objects:
                    class_name = getattr(obj, 'class_name', f"class_{obj.class_id}")
                    self.metric_detections_total.labels(
                        class_id=str(obj.class_id),
                        class_name=class_name,
                        mission_id=self.current_mission_id
                    ).inc()
                    
                    # Update confidence histogram
                    self.metric_confidence_histogram.labels(
                        class_id=str(obj.class_id),
                        mission_id=self.current_mission_id
                    ).observe(obj.confidence)
            else:
                self.logger.debug("no_objects_detected", extra={
                    "frame_id": self.detection_attempts
                })

        except Exception as e:
            self.error_count += 1
            self.logger.exception("image_processing_failed", extra={
                "frame_id": self.detection_attempts,
                "error": str(e)
            })
            self.error_handler.handle_error(e, "Image processing failed")
            self.metric_errors_total.labels(
                error_type="processing_error",
                mission_id=self.current_mission_id
            ).inc()
            if self.debug_mode:
                traceback.print_exc()

        # Track performance
        elapsed = time.time() - start_time
        self.frame_times.append(elapsed)
        self.total_detection_time += elapsed
        
        # Update frames processed counter
        self.metric_frames_processed_total.labels(
            mission_id=self.current_mission_id
        ).inc()

    def _run_inference(self, cv_img):
        """Run YOLO inference"""
        try:
            self.logger.debug("inference_started", extra={
                "image_shape": cv_img.shape,
                "conf_threshold": self.conf_threshold,
                "input_size": self.input_size
            })

            # Run inference
            results = self.model(
                cv_img,
                verbose=False,
                conf=self.conf_threshold,
                iou=self.nms_threshold,
                imgsz=self.input_size,
                device=self.device,
            )

            # Parse results
            det = DetectedObjects()
            det.header.stamp = rospy.Time.now()
            det.header.frame_id = 'camera_frame'
            det.objects = []

            for r in results[0].boxes:
                cls_id = int(r.cls[0].item())
                if cls_id not in self.vehicle_classes:
                    continue

                conf = r.conf[0].item()
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = r.xyxy[0].tolist()

                bbox = BBox()
                bbox.x = x1
                bbox.y = y1
                bbox.width = x2 - x1
                bbox.height = y2 - y1

                obj = Object()
                obj.class_id = cls_id
                obj.class_name = self.model.names[cls_id]
                obj.confidence = conf
                obj.bbox = bbox
                det.objects.append(obj)

            self.logger.debug("inference_completed", extra={
                "detections_found": len(det.objects)
            })

            return det

        except Exception as e:
            self.logger.exception("inference_failed", extra={
                "error": str(e)
            })
            self.error_handler.handle_error(e, "Inference failed")
            empty = DetectedObjects()
            empty.header.stamp = rospy.Time.now()
            empty.header.frame_id = 'camera_frame'
            empty.objects = []
            return empty

    def _simulate_detections(self, cv_img):
        """Simulate detections for test mode"""
        import random

        self.logger.debug("simulation_detection_started", extra={
            "mock_accuracy": self.mock_accuracy,
            "false_positive_rate": self.false_positive_rate
        })

        det = DetectedObjects()
        det.header.stamp = rospy.Time.now()
        det.header.frame_id = 'camera_frame'
        det.objects = []

        # Add simulated detections based on mock accuracy
        if random.random() < self.mock_accuracy:
            num_objects = random.randint(1, 3)
            for i in range(num_objects):
                obj = Object()
                obj.class_id = random.choice(self.vehicle_classes)
                obj.class_name = self.model.names[obj.class_id] if hasattr(self.model, 'names') else f"class_{obj.class_id}"
                obj.confidence = random.uniform(0.6, 0.98)

                # Random bounding box
                img_h, img_w = cv_img.shape[:2]
                x = random.uniform(0, img_w - 100)
                y = random.uniform(0, img_h - 100)
                w = random.uniform(30, 100)
                h = random.uniform(30, 100)

                obj.bbox = BBox()
                obj.bbox.x = x
                obj.bbox.y = y
                obj.bbox.width = w
                obj.bbox.height = h
                det.objects.append(obj)

        # Add false positives if configured
        if random.random() < self.false_positive_rate:
            self.false_positives += 1
            self.metric_false_positives_total.labels(
                mission_id=self.current_mission_id
            ).inc()
            
            self.logger.warning("false_positive_detected", extra={
                "frame_id": self.detection_attempts
            })
            
            fp_obj = Object()
            fp_obj.class_id = 0  # Unknown
            fp_obj.class_name = "false_positive"
            fp_obj.confidence = random.uniform(0.3, 0.5)

            img_h, img_w = cv_img.shape[:2]
            fp_obj.bbox = BBox()
            fp_obj.bbox.x = random.uniform(0, img_w - 50)
            fp_obj.bbox.y = random.uniform(0, img_h - 50)
            fp_obj.bbox.width = random.uniform(20, 60)
            fp_obj.bbox.height = random.uniform(20, 60)
            det.objects.append(fp_obj)

        self.logger.debug("simulation_detection_completed", extra={
            "objects_found": len(det.objects)
        })

        return det

    def _publish_health(self, event=None):
        """Publish node health status with metrics"""
        is_healthy = self.node_health == "running" and self.error_count < 10
        
        # Update health status gauge
        self.metric_health_status.labels(
            mission_id=self.current_mission_id
        ).set(1 if is_healthy else 0)
        
        # Log health status changes
        if not is_healthy:
            self.logger.warning("node_unhealthy", extra={
                "node_health": self.node_health,
                "error_count": self.error_count,
                "detection_count": self.detection_count
            })
        
        health_msg = NodeHealth()
        health_msg.node_name = 'yolo_detector'
        health_msg.status = self.node_health
        health_msg.timestamp = rospy.Time.now()
        health_msg.detection_count = self.detection_count
        health_msg.error_count = self.error_count
        health_msg.fps = self.fps
        health_msg.is_healthy = is_healthy
        health_msg.cpu_usage = psutil.cpu_percent()
        health_msg.memory_usage = psutil.virtual_memory().percent

        self.health_pub.publish(health_msg)
        self.logger.debug("health_published", extra={
            "is_healthy": is_healthy,
            "fps": self.fps,
            "error_count": self.error_count
        })


if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass