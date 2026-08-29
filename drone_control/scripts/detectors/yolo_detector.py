#!/usr/bin/env python3
"""
drone_control/scripts/detectors/yolo_detector.py
Enhanced YOLO detector with validation, error handling, and health monitoring
"""

import rospy
import cv2
import numpy as np
import os
import sys
import time
from sensor_msgs.msg import Image
from drone_control.msg import DetectedObjects, Object, BBox
from drone_control.msg import NodeHealth, Command, CommandResponse, MissionStatus
from cv_bridge import CvBridge
from ultralytics import YOLO
import traceback

# Add utils path
sys.path.append(os.path.join(os.path.dirname(__file__), 'utils'))
try:
    from error_handler import ErrorHandler
    from message_validator import MessageValidator
except ImportError:
    # Fallback if utils not yet created
    class ErrorHandler:
        def __init__(self, node_name):
            self.node_name = node_name
        def handle_error(self, error, context=None):
            rospy.logerr(f"[{self.node_name}] {error}: {context}")
    class MessageValidator:
        def validate_image(self, msg):
            return True, "OK"

class YOLODetector:
    """Enhanced YOLO detector with full system integration"""

    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=False)

        # Setup components
        self.bridge = CvBridge()
        self.error_handler = ErrorHandler(node_name='yolo_detector')
        self.message_validator = MessageValidator()

        # Get parameters
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

        # State variables
        self.node_health = "running"
        self.last_health_publish = time.time()
        self.detection_count = 0
        self.error_count = 0
        self.fps = 0.0
        self.frame_times = []

        # Health monitoring (single publisher + timer, created once)
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

        rospy.loginfo(f"YOLO Detector initialized in {'test' if self.test_mode else 'production'} mode")
        rospy.loginfo(f"Vehicle classes: {self.vehicle_classes}")

    def _load_model(self):
        """Load YOLO model with validation"""
        try:
            rospy.loginfo(f"Loading YOLO model from: {self.model_path}")

            # Check if model exists
            if not os.path.exists(self.model_path) and self.model_path != 'yolov8s.pt':
                raise FileNotFoundError(f"Model not found: {self.model_path}")

            # Load model
            model = YOLO(self.model_path)

            # Validate model loaded
            if model is None:
                raise RuntimeError("Failed to load YOLO model")

            # Move model onto the configured device (was previously ignored)
            try:
                model.to(self.device)
            except Exception as device_err:
                rospy.logwarn(f"Could not move model to device '{self.device}': {device_err}")

            rospy.loginfo("YOLO model loaded successfully!")
            return model

        except Exception as e:
            self.error_handler.handle_error(e, "Model loading failed")
            rospy.logerr(f"Failed to load model: {e}")
            rospy.signal_shutdown(f"Model loading failed: {e}")
            sys.exit(1)

    def _handle_command(self, msg):
        """Handle incoming commands"""
        rospy.logdebug(f"Received command: {msg.command}")

        try:
            if msg.command == 'reload_model':
                self._reload_model()
                self._send_response(msg.command, True, "Model reloaded")
            elif msg.command == 'set_confidence':
                self.conf_threshold = msg.parameters.get('threshold', 0.5)
                self._send_response(msg.command, True, f"Confidence set to {self.conf_threshold}")
            elif msg.command == 'enable_debug':
                self.debug_mode = True
                self._send_response(msg.command, True, "Debug mode enabled")
            elif msg.command == 'disable_debug':
                self.debug_mode = False
                self._send_response(msg.command, True, "Debug mode disabled")
            else:
                self._send_response(msg.command, False, f"Unknown command: {msg.command}")

        except Exception as e:
            self.error_handler.handle_error(e, f"Command: {msg.command}")
            self._send_response(msg.command, False, str(e))

    def _send_response(self, command, success, message):
        """Send command response"""
        response = CommandResponse()
        response.original_command = command
        response.success = success
        response.message = message
        self.response_pub.publish(response)

    def _handle_mission_status(self, msg):
        """Handle mission status updates"""
        self.current_mission_state = msg.state
        if self.debug_mode:
            rospy.logdebug(f"Mission state: {self.current_mission_state}")

    def _reload_model(self):
        """Reload the model at runtime"""
        try:
            rospy.loginfo("Reloading model...")
            model = YOLO(self.model_path)
            try:
                model.to(self.device)
            except Exception as device_err:
                rospy.logwarn(f"Could not move reloaded model to device '{self.device}': {device_err}")
            self.model = model
            rospy.loginfo("Model reloaded successfully")
        except Exception as e:
            self.error_handler.handle_error(e, "Model reload failed")
            raise

    def _validate_image(self, msg):
        """Validate incoming image message"""
        is_valid, error = self.message_validator.validate_image(msg)
        if not is_valid:
            rospy.logwarn(f"Invalid image message: {error}")
        return is_valid, error

    def _publish_performance(self, event):
        """Publish performance metrics"""
        if self.frame_times:
            total_time = sum(self.frame_times)
            avg_fps = len(self.frame_times) / total_time if total_time > 0 else 0
            self.fps = avg_fps
            rospy.logdebug(f"Detector FPS: {avg_fps:.1f}, Detections: {self.detection_count}")
            self.frame_times = []

    def image_callback(self, msg):
        """Process incoming image"""
        start_time = time.time()

        try:
            # Validate message
            is_valid, error = self._validate_image(msg)
            if not is_valid:
                self.error_count += 1
                return

            # Convert ROS image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Test mode - simulate detections
            if self.test_mode:
                detections = self._simulate_detections(cv_img)
            else:
                # Run inference
                detections = self._run_inference(cv_img)

            # Publish detections only when we actually have a valid, non-empty result.
            # (Previously `if detections:` was always truthy for a ROS message object,
            # so this branch ran even when inference failed and returned an empty message.)
            if detections is not None and len(detections.objects) > 0:
                self.objects_pub.publish(detections)
                self.detection_count += len(detections.objects)

        except Exception as e:
            self.error_count += 1
            self.error_handler.handle_error(e, "Image processing failed")
            rospy.logwarn(f"Error processing image: {e}")
            if self.debug_mode:
                traceback.print_exc()

        # Track performance
        elapsed = time.time() - start_time
        self.frame_times.append(elapsed)

    def _run_inference(self, cv_img):
        """Run YOLO inference"""
        try:
            # Run inference, now actually using the configured device,
            # NMS/IoU threshold, and input size instead of ultralytics defaults.
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

            if self.debug_mode and det.objects:
                rospy.logdebug(f"Detected {len(det.objects)} objects")

            return det

        except Exception as e:
            self.error_handler.handle_error(e, "Inference failed")
            empty = DetectedObjects()
            empty.header.stamp = rospy.Time.now()
            empty.header.frame_id = 'camera_frame'
            empty.objects = []
            return empty

    def _simulate_detections(self, cv_img):
        """Simulate detections for test mode"""
        import random

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

        return det

    def _publish_health(self, event=None):
        """Publish node health status"""
        health_msg = NodeHealth()
        health_msg.node_name = 'yolo_detector'
        health_msg.status = self.node_health
        health_msg.timestamp = rospy.Time.now()
        health_msg.detection_count = self.detection_count
        health_msg.error_count = self.error_count
        health_msg.fps = self.fps
        health_msg.is_healthy = self.node_health == "running" and self.error_count < 10
          # Add CPU and memory usage
        import psutil
        health_msg.cpu_usage = psutil.cpu_percent()
        health_msg.memory_usage = psutil.virtual_memory().percent

        self.health_pub.publish(health_msg)


if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass