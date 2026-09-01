#!/usr/bin/env python3
"""
drone_control/scripts/detectors/object_tracker.py
Object tracking with Kalman filter and association
"""

import numpy as np
import rospy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter

from drone_control.msg import (
    Detection,
    DetectionArray,
    NodeHealth,
    TrackedTarget,
    TrackedTargets,
)
from drone_control.utils import ErrorHandler


class Track:
    """Single track representation"""
    
    def __init__(self, detection, track_id):
        self.track_id = track_id
        self.class_id = detection.class_id
        self.confidence = detection.confidence
        
        # Initialize Kalman filter
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        dt = 0.1
        
        self.kf.F = np.array([
            [1,0,0,dt,0,0],
            [0,1,0,0,dt,0],
            [0,0,1,0,0,dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])
        self.kf.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]
        ])
        self.kf.P *= 10.0
        self.kf.R = np.eye(3) * 0.05
        self.kf.Q = np.eye(6) * 0.1
        
        # Initialize position
        pos = np.array([detection.x, detection.y, detection.z])
        self.kf.x = np.array([pos[0], pos[1], pos[2], 0, 0, 0])
        
        self.hit_count = 1
        self.miss_count = 0
        self.last_update = rospy.Time.now()
        
    def predict(self):
        """Predict next state"""
        self.kf.predict()
        
    def update(self, detection):
        """Update track with detection"""
        pos = np.array([detection.x, detection.y, detection.z])
        self.kf.update(pos)
        self.hit_count += 1
        self.confidence = detection.confidence
        self.last_update = rospy.Time.now()
        
    def get_position(self):
        """Get current position"""
        return self.kf.x[:3]
        
    def get_velocity(self):
        """Get current velocity"""
        return self.kf.x[3:6]
        
    def get_confidence(self):
        """Get track confidence"""
        return min(1.0, self.hit_count / (self.hit_count + self.miss_count + 1))

class ObjectTracker:
    """Object tracking with Kalman filter and association"""
    
    def __init__(self):
        rospy.init_node('object_tracker', anonymous=False)
        
        self.error_handler = ErrorHandler(node_name='object_tracker')
        
        # Parameters
        self.max_tracks = rospy.get_param('tracking/max_tracks', 20)
        self.max_lost_frames = rospy.get_param('tracking/max_lost_frames', 10)
        self.iou_threshold = rospy.get_param('tracking/association/iou_threshold', 0.3)
        self.distance_threshold = rospy.get_param('tracking/association/distance_threshold', 1.0)
        
        # State
        self.tracks = []
        self.next_track_id = 0
        self.last_detections = []
        
        # Subscribers
        rospy.Subscriber('/detected_objects', DetectionArray, self._detections_callback)
        
        # Publishers
        self.tracks_pub = rospy.Publisher('/tracked_targets', TrackedTargets, queue_size=10)
        self.health_pub = rospy.Publisher('/object_tracker/node_health', NodeHealth, queue_size=10)
        
        # Timer
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self._update)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._publish_health)
        
        rospy.loginfo("Object Tracker initialized")
        
    def _detections_callback(self, msg):
        """Handle detection messages"""
        self.last_detections = msg.detections
        
    def _update(self, event):
        """Update tracker"""
        # Predict tracks
        for track in self.tracks:
            track.predict()
            track.miss_count += 1
            
        # Associate detections with tracks
        if self.last_detections:
            self._associate_detections(self.last_detections)
            
        # Remove stale tracks
        stale_tracks = []
        for track in self.tracks:
            if track.miss_count > self.max_lost_frames:
                stale_tracks.append(track)
                
        for track in stale_tracks:
            self.tracks.remove(track)
            rospy.logdebug(f"Removed stale track: {track.track_id}")
            
        # Publish tracks
        self._publish_tracks()
        
        # Clear detections
        self.last_detections = []
        
    def _associate_detections(self, detections):
        """Associate detections with existing tracks"""
        # Simple IoU-based association
        matched_tracks = []
        matched_detections = []
        
        # Compute IoU matrix
        iou_matrix = np.zeros((len(self.tracks), len(detections)))
        for i, track in enumerate(self.tracks):
            track_pos = track.get_position()
            for j, det in enumerate(detections):
                det_pos = np.array([det.x, det.y, det.z])
                distance = np.linalg.norm(track_pos - det_pos)
                iou_matrix[i, j] = 1.0 / (distance + 1.0)
                
        # Hungarian algorithm (simplified greedy matching)
        while len(matched_tracks) < len(self.tracks) and len(matched_detections) < len(detections):
            best_i = -1
            best_j = -1
            best_score = 0
            
            for i in range(len(self.tracks)):
                if i in matched_tracks:
                    continue
                for j in range(len(detections)):
                    if j in matched_detections:
                        continue
                    if iou_matrix[i, j] > best_score:
                        best_score = iou_matrix[i, j]
                        best_i = i
                        best_j = j
                        
            if best_score > self.iou_threshold:
                matched_tracks.append(best_i)
                matched_detections.append(best_j)
                self.tracks[best_i].update(detections[best_j])
            else:
                break
                
        # Create new tracks for unmatched detections
        for j in range(len(detections)):
            if j not in matched_detections:
                new_track = Track(detections[j], self.next_track_id)
                self.next_track_id += 1
                self.tracks.append(new_track)
                rospy.logdebug(f"Created new track: {new_track.track_id}")
                
    def _publish_tracks(self):
        """Publish tracked targets"""
        targets_msg = TrackedTargets()
        targets_msg.header.stamp = rospy.Time.now()
        targets_msg.header.frame_id = 'map'
        targets_msg.count = len(self.tracks)
        
        for track in self.tracks:
            target = TrackedTarget()
            target.id = track.track_id
            position = track.get_position()
            velocity = track.get_velocity()
            
            target.position.x = position[0]
            target.position.y = position[1]
            target.position.z = position[2]
            target.velocity.x = velocity[0]
            target.velocity.y = velocity[1]
            target.velocity.z = velocity[2]
            target.confidence = track.get_confidence()
            
            targets_msg.targets.append(target)
            
        self.tracks_pub.publish(targets_msg)
        
    def _publish_health(self, event):
        """Publish node health"""
        health_msg = NodeHealth()
        health_msg.node_name = 'object_tracker'
        health_msg.status = 'running'
        health_msg.timestamp = rospy.Time.now()
        health_msg.detection_count = len(self.tracks)
        health_msg.is_healthy = True
          # Add CPU and memory usage
        import psutil
        health_msg.cpu_usage = psutil.cpu_percent()
        health_msg.memory_usage = psutil.virtual_memory().percent

        self.health_pub.publish(health_msg)

if __name__ == '__main__':
    try:
        tracker = ObjectTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass