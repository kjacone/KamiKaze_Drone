#!/usr/bin/env python3
# drone_control/test/test_tracker.py

import unittest

import numpy as np
import rospy
from drone_control import KalmanTracker, TargetTracker
from filterpy.kalman import KalmanFilter

from drone_control import Detection, TrackedTarget, TrackedTargets


class TestKalmanTracker(unittest.TestCase):
    """Unit tests for Kalman Filter tracking"""
    
    def setUp(self):
        """Initialize Kalman tracker before each test"""
        self.tracker = KalmanTracker()
        self.target_id = 0
        
    def test_initialization(self):
        """Test Kalman filter initialization"""
        self.assertIsNotNone(self.tracker.kf)
        self.assertEqual(self.tracker.kf.dim_x, 6)  # [x, y, z, vx, vy, vz]
        self.assertEqual(self.tracker.kf.dim_z, 3)  # [x, y, z]
        
    def test_predict_update(self):
        """Test prediction and update cycle"""
        # Initial measurement
        measurement = np.array([10.0, 5.0, 2.0])
        self.tracker.initialize(measurement)
        
        # Predict
        predicted = self.tracker.predict()
        self.assertEqual(predicted.shape, (3,))
        
        # Update with new measurement
        new_measurement = np.array([10.5, 5.2, 2.1])
        updated = self.tracker.update(new_measurement)
        self.assertEqual(updated.shape, (3,))
        
        # Check state converges
        self.assertLess(np.linalg.norm(predicted - new_measurement), 1.0)
        
    def test_motion_models(self):
        """Test different motion models"""
        # Constant velocity
        track = self.tracker
        measurements = []
        for i in range(10):
            measurements.append(np.array([i, i*0.5, 2.0]))
            
        for i, meas in enumerate(measurements):
            if i == 0:
                track.initialize(meas)
            else:
                track.predict()
                track.update(meas)
                
        # Check velocity estimation
        final_state = track.kf.x
        self.assertAlmostEqual(final_state[3], 1.0, delta=0.1)  # vx
        self.assertAlmostEqual(final_state[4], 0.5, delta=0.1)  # vy
        
    def test_track_management(self):
        """Test track creation and management"""
        target_tracker = TargetTracker()
        
        # Create detections
        detections = [
            Detection(bbox=[0, 0, 10, 10], confidence=0.9, class_id=0),
            Detection(bbox=[20, 10, 10, 10], confidence=0.8, class_id=0),
            Detection(bbox=[-5, -5, 10, 10], confidence=0.7, class_id=1)
        ]
        
        # Update tracker
        tracked = target_tracker.update(detections)
        
        # Should create 3 tracks
        self.assertEqual(len(tracked.targets), 3)
        self.assertEqual(tracked.targets[0].id, 1)
        
    def test_track_lost_recovery(self):
        """Test track recovery after lost frames"""
        target_tracker = TargetTracker()
        target_tracker.max_lost_frames = 5
        
        # Create initial track
        det = Detection(bbox=[0, 0, 10, 10], confidence=0.9, class_id=0)
        target_tracker.update([det])
        self.assertEqual(len(target_tracker.tracks), 1)
        
        # Lose track for several frames
        for _ in range(3):
            target_tracker.update([])
            self.assertEqual(len(target_tracker.tracks), 1)  # Still tracking
            
        # Lose track beyond threshold
        for _ in range(3):
            target_tracker.update([])
            
        # Track should be removed
        self.assertEqual(len(target_tracker.tracks), 0)
        
    def test_association(self):
        """Test detection-to-track association"""
        target_tracker = TargetTracker()
        
        # Create two tracks
        for i in range(2):
            det = Detection(
                bbox=[i*10, 0, 10, 10], 
                confidence=0.9, 
                class_id=0
            )
            target_tracker.update([det])
            
        # New detections close to existing tracks
        new_dets = [
            Detection(bbox=[12, 2, 10, 10], confidence=0.8, class_id=0),
            Detection(bbox=[22, 1, 10, 10], confidence=0.9, class_id=0)
        ]
        
        # Update - should associate with existing tracks
        tracked = target_tracker.update(new_dets)
        self.assertEqual(len(tracked.targets), 2)
        
        # Track IDs should remain consistent
        self.assertEqual(tracked.targets[0].id, 1)
        self.assertEqual(tracked.targets[1].id, 2)
        
    def test_kinematic_properties(self):
        """Test calculation of kinematic properties"""
        import math
        
        tracked = TrackedTarget()
        tracked.position = [0, 0, 5]
        tracked.velocity = [10, 0, 0]
        
        # Test speed
        speed = math.sqrt(sum(v**2 for v in tracked.velocity))
        self.assertEqual(speed, 10.0)
        
        # Test heading
        heading = math.atan2(tracked.velocity[1], tracked.velocity[0])
        self.assertEqual(heading, 0.0)
        
    @unittest.skipIf(not rospy.has_param('/test_mode_active'), "Test mode not active")
    def test_publish_subscribe(self):
        """Test tracking node with ROS messages"""
        import rospy
        from drone_control import DetectionArray
        
        # Wait for topic
        try:
            rospy.wait_for_message('/tracked_targets', TrackedTargets, timeout=10.0)
        except rospy.ROSException:
            self.fail("Tracked targets topic not publishing")
            
        # Check message structure
        msg = rospy.wait_for_message('/tracked_targets', TrackedTargets, timeout=10.0)
        self.assertIsNotNone(msg)
        
        # Verify message fields
        for target in msg.targets:
            self.assertTrue(hasattr(target, 'id'))
            self.assertTrue(hasattr(target, 'position'))
            self.assertTrue(hasattr(target, 'velocity'))
            self.assertTrue(hasattr(target, 'confidence'))
            self.assertTrue(0 <= target.confidence <= 1.0)

if __name__ == '__main__':
    unittest.main(verbosity=2)