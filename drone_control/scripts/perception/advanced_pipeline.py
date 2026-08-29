#!/usr/bin/env python3
"""
Advanced Perception Pipeline
Multi-sensor fusion (LiDAR + Camera + Radar) with deep learning-based object detection
"""

import rospy
import cv2
import numpy as np
import torch
import torchvision
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import Point, Pose, Quaternion
from drone_control.msg import DetectedObjects, Detection
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import message_filters

class AdvancedPerceptionPipeline:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('advanced_perception_pipeline', anonymous=True)
        
        # Initialize bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        
        # Get parameters
        self.sensor_fusion_enabled = rospy.get_param('~sensor_fusion_enabled', True)
        self.use_deep_learning = rospy.get_param('~use_deep_learning', True)
        self.model_path = rospy.get_param('~model_path', '/root/PX4-Autopilot/yolov8s.pt')
        self.device = rospy.get_param('~device', 'cpu')
        self.publish_rate = rospy.get_param('~publish_rate', 10)
        
        # Load deep learning model
        if self.use_deep_learning:
            self.load_model()
        
        # Initialize sensor fusion
        self.fusion_weights = {
            'camera': 0.4,
            'lidar': 0.4,
            'radar': 0.2
        }
        
        # Setup subscribers with synchronization
        self.setup_subscribers()
        
        # Setup publishers
        self.detection_pub = rospy.Publisher('/perception/detected_objects', 
                                            DetectedObjects, queue_size=10)
        self.visualization_pub = rospy.Publisher('/perception/visualization', 
                                                 Image, queue_size=10)
        
        rospy.loginfo("Advanced Perception Pipeline initialized")
    
    def load_model(self):
        """Load the deep learning model for object detection"""
        try:
            # Load YOLO model using ultralytics
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
            rospy.loginfo(f"Model loaded from {self.model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load model: {e}")
            self.model = None
            self.use_deep_learning = False
    
    def setup_subscribers(self):
        """Setup synchronized subscribers for multi-sensor fusion"""
        # Camera subscriber
        self.camera_sub = message_filters.Subscriber(
            '/camera/color/image_raw', Image)
        
        # LiDAR subscriber (if available)
        self.lidar_sub = message_filters.Subscriber(
            '/velodyne_points', PointCloud2)
        
        # Radar subscriber (if available)
        self.radar_sub = message_filters.Subscriber(
            '/radar/points', PointCloud2)
        
        # Time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.camera_sub, self.lidar_sub, self.radar_sub], 
            queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sensor_callback)
    
    def sensor_callback(self, camera_msg, lidar_msg, radar_msg):
        """Process synchronized sensor data"""
        try:
            # Process camera image
            camera_objects = self.process_camera(camera_msg)
            
            # Process LiDAR point cloud
            lidar_objects = self.process_lidar(lidar_msg)
            
            # Process radar point cloud
            radar_objects = self.process_radar(radar_msg)
            
            # Fuse detections
            if self.sensor_fusion_enabled:
                fused_objects = self.fuse_detections(
                    camera_objects, lidar_objects, radar_objects)
            else:
                fused_objects = camera_objects
            
            # Publish fused detections
            self.publish_detections(fused_objects, camera_msg.header)
            
            # Publish visualization
            if rospy.get_param('~visualize', False):
                self.publish_visualization(camera_msg, fused_objects)
                
        except Exception as e:
            rospy.logerr(f"Error in sensor callback: {e}")
    
    def process_camera(self, camera_msg):
        """Process camera image for object detection"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
            
            if self.use_deep_learning and self.model is not None:
                # Run YOLO inference
                results = self.model(cv_image, conf=0.3)
                
                # Extract detections
                detections = []
                for result in results:
                    boxes = result.boxes
                    if boxes is not None:
                        for box in boxes:
                            detection = {
                                'type': 'camera',
                                'class': int(box.cls.item()),
                                'confidence': float(box.conf.item()),
                                'bbox': box.xyxy[0].tolist(),
                                'center': [
                                    float((box.xyxy[0][0] + box.xyxy[0][2]) / 2),
                                    float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                                ],
                                'size': [
                                    float(box.xyxy[0][2] - box.xyxy[0][0]),
                                    float(box.xyxy[0][3] - box.xyxy[0][1])
                                ]
                            }
                            detections.append(detection)
                return detections
            
            return []
            
        except Exception as e:
            rospy.logerr(f"Error processing camera: {e}")
            return []
    
    def process_lidar(self, lidar_msg):
        """Process LiDAR point cloud for object detection"""
        try:
            if lidar_msg is None or len(lidar_msg.data) == 0:
                return []
            
            # Convert point cloud to numpy array
            points = []
            for p in pc2.read_points(lidar_msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if len(points) == 0:
                return []
            
            points = np.array(points)
            
            # Simple clustering-based detection
            # For production, use DBSCAN or similar clustering
            detections = []
            
            # This is a simplified example - replace with actual clustering algorithm
            if len(points) > 10:
                # Just create a single detection for demo
                center = np.mean(points, axis=0)
                detection = {
                    'type': 'lidar',
                    'class': 0,  # Unknown
                    'confidence': 0.8,
                    'position': center.tolist(),
                    'size': [1.0, 1.0, 1.0]  # Estimate size
                }
                detections.append(detection)
            
            return detections
            
        except Exception as e:
            rospy.logerr(f"Error processing LiDAR: {e}")
            return []
    
    def process_radar(self, radar_msg):
        """Process radar data for object detection"""
        try:
            if radar_msg is None or len(radar_msg.data) == 0:
                return []
            
            # Similar to LiDAR processing
            points = []
            for p in pc2.read_points(radar_msg, field_names=('x', 'y', 'z'), skip_nans=True):
                points.append([p[0], p[1], p[2]])
            
            if len(points) == 0:
                return []
            
            points = np.array(points)
            
            # Process radar detections
            detections = []
            for point in points:
                detection = {
                    'type': 'radar',
                    'class': 0,
                    'confidence': 0.7,
                    'position': point.tolist(),
                    'velocity': [0, 0, 0]  # Estimate velocity
                }
                detections.append(detection)
            
            return detections
            
        except Exception as e:
            rospy.logerr(f"Error processing radar: {e}")
            return []
    
    def fuse_detections(self, camera_objects, lidar_objects, radar_objects):
        """Fuse detections from multiple sensors"""
        fused_objects = []
        
        # Convert to common format and apply weights
        for obj in camera_objects:
            obj['weight'] = self.fusion_weights['camera'] * obj['confidence']
            fused_objects.append(obj)
        
        for obj in lidar_objects:
            obj['weight'] = self.fusion_weights['lidar'] * obj['confidence']
            fused_objects.append(obj)
        
        for obj in radar_objects:
            obj['weight'] = self.fusion_weights['radar'] * obj['confidence']
            fused_objects.append(obj)
        
        # Perform non-maximum suppression for overlapping detections
        if len(fused_objects) > 0:
            # Sort by weight
            fused_objects.sort(key=lambda x: x['weight'], reverse=True)
            
            # NMS based on position (simplified)
            filtered = []
            for obj in fused_objects:
                keep = True
                for filtered_obj in filtered:
                    # Check if objects are close
                    if 'center' in obj and 'center' in filtered_obj:
                        dist = np.linalg.norm(
                            np.array(obj['center']) - np.array(filtered_obj['center']))
                        if dist < 50:  # pixels
                            keep = False
                            break
                    elif 'position' in obj and 'position' in filtered_obj:
                        dist = np.linalg.norm(
                            np.array(obj['position']) - np.array(filtered_obj['position']))
                        if dist < 0.5:  # meters
                            keep = False
                            break
                if keep:
                    filtered.append(obj)
            
            return filtered
        
        return fused_objects
    
    def publish_detections(self, detections, header):
        """Publish detected objects as ROS message"""
        msg = DetectedObjects()
        msg.header = header
        
        for i, det in enumerate(detections):
            detection_msg = Detection()
            detection_msg.id = i
            detection_msg.class_id = det.get('class', 0)
            detection_msg.confidence = det.get('confidence', 0.0)
            
            # Set position (for 3D detections)
            if 'position' in det:
                detection_msg.position.x = det['position'][0]
                detection_msg.position.y = det['position'][1]
                detection_msg.position.z = det['position'][2] if len(det['position']) > 2 else 0.0
            
            # Set bounding box (for 2D detections)
            if 'bbox' in det:
                detection_msg.bbox.x = det['bbox'][0]
                detection_msg.bbox.y = det['bbox'][1]
                detection_msg.bbox.width = det['bbox'][2] - det['bbox'][0]
                detection_msg.bbox.height = det['bbox'][3] - det['bbox'][1]
            
            msg.detections.append(detection_msg)
        
        self.detection_pub.publish(msg)
    
    def publish_visualization(self, camera_msg, detections):
        """Publish visualization of detections"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
            
            for det in detections:
                if 'bbox' in det:
                    bbox = det['bbox']
                    cv2.rectangle(cv_image, 
                                 (int(bbox[0]), int(bbox[1])),
                                 (int(bbox[2]), int(bbox[3])),
                                 (0, 255, 0), 2)
                    
                    # Add label
                    label = f"Class {det['class']}: {det['confidence']:.2f}"
                    cv2.putText(cv_image, label,
                               (int(bbox[0]), int(bbox[1] - 10)),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            vis_msg.header = camera_msg.header
            self.visualization_pub.publish(vis_msg)
            
        except Exception as e:
            rospy.logerr(f"Error publishing visualization: {e}")
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        pipeline = AdvancedPerceptionPipeline()
        pipeline.run()
    except rospy.ROSInterruptException:
        pass