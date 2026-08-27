#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from drone_control.msg import DetectedObjects, Object, BBox
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Load YOLO model using ultralytics API (more reliable)
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO('yolov8s.pt')  # This will download automatically
        rospy.loginfo("YOLO model loaded successfully!")
        
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.vehicle_classes = rospy.get_param('vehicle_classes', [2, 3, 5, 6, 7])
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.objects_pub = rospy.Publisher('/detected_objects', DetectedObjects, queue_size=10)
        rospy.loginfo("YOLO Detector initialized")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"CV Bridge conversion failed: {e}")
            return

        # Run inference
        results = self.model(cv_img, verbose=False)
        det = DetectedObjects()
        det.header = msg.header
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

        if det.objects:
            rospy.loginfo(f"Detected {len(det.objects)} objects")
            self.objects_pub.publish(det)

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
