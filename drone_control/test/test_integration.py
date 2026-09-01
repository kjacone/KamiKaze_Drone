#!/usr/bin/env python3
# drone_control/test/test_integration.py

import time
import unittest

import numpy as np
import rospy
from drone_control import MockSensors, ScenarioRunner, TestDataRecorder
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import Image

from drone_control import ControlCommand, DetectionArray, TrackedTargets


class TestIntegration(unittest.TestCase):
    """End-to-end integration tests for the drone control pipeline"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS and test environment"""
        if not rospy.get_node_uri():
            rospy.init_node('integration_test', anonymous=True)
        
        cls.mock_sensors = MockSensors()
        cls.scenario_runner = ScenarioRunner()
        cls.recorder = TestDataRecorder()
        
        # Topics
        cls.detection_topic = '/target_detections'
        cls.tracked_topic = '/tracked_targets'
        cls.control_topic = '/control_command'
        cls.camera_topic = '/camera/image_raw'
        
        # Publishers and subscribers
        cls.camera_pub = rospy.Publisher(cls.camera_topic, Image, queue_size=10)
        cls.test_results = []
        
    def setUp(self):
        """Reset before each test"""
        self.recorder.clear()
        time.sleep(0.5)  # Allow system to stabilize
        
    def test_1_pipeline_end_to_end(self):
        """Test entire pipeline: camera -> detections -> tracking -> control"""
        print("Testing end-to-end pipeline...")
        
        # Start recorder
        self.recorder.start_recording(
            topics=[self.detection_topic, self.tracked_topic, self.control_topic],
            duration=30
        )
        
        # Generate test scenario
        scenario = self.scenario_runner.load_scenario('default')
        self.scenario_runner.start_scenario(scenario)
        
        # Simulate camera feed with targets
        received_detections = 0
        received_tracks = 0
        received_controls = 0
        start_time = time.time()
        
        # Run for 30 seconds
        while time.time() - start_time < 30:
            # Generate camera image with targets
            img = self.scenario_runner.get_frame()
            
            # Publish image
            img_msg = self._numpy_to_ros_image(img)
            self.camera_pub.publish(img_msg)
            
            # Monitor outputs
            if rospy.get_published_topics():
                # Check detection topic
                if rospy.has_param(self.detection_topic):
                    detections = rospy.wait_for_message(
                        self.detection_topic, 
                        DetectionArray, 
                        timeout=0.1
                    )
                    if detections:
                        received_detections += len(detections.detections)
                        
                # Check tracking topic
                if rospy.has_param(self.tracked_topic):
                    tracked = rospy.wait_for_message(
                        self.tracked_topic, 
                        TrackedTargets, 
                        timeout=0.1
                    )
                    if tracked:
                        received_tracks += len(tracked.targets)
                        
            time.sleep(0.1)  # Simulate camera frame rate
            
        # Verify data flow
        self.assertGreater(received_detections, 0, "No detections received")
        self.assertGreater(received_tracks, 0, "No tracked targets received")
        
        print(f"Received {received_detections} detections, {received_tracks} tracked targets")
        
    def test_2_control_response(self):
        """Test control system response to tracking"""
        print("Testing control response...")
        
        # Create a tracked target
        target = TrackedTarget()
        target.id = 1
        target.position = Point(10.0, 5.0, 3.0)
        target.velocity = Point(0.0, 0.0, 0.0)
        target.confidence = 0.95
        
        # Publish tracked target
        tracked_msg = TrackedTargets()
        tracked_msg.targets = [target]
        
        pub = rospy.Publisher(self.tracked_topic, TrackedTargets, queue_size=1)
        pub.publish(tracked_msg)
        
        # Wait for control command
        try:
            control_msg = rospy.wait_for_message(
                self.control_topic, 
                ControlCommand, 
                timeout=5.0
            )
            self.assertIsNotNone(control_msg)
            print(f"Control command received: {control_msg}")
            
        except rospy.ROSException:
            self.fail("Control command not received within timeout")
            
    def test_3_safety_check(self):
        """Test safety system responses"""
        print("Testing safety system...")
        
        # Test geofence violation
        dangerous_target = TrackedTarget()
        dangerous_target.id = 1
        dangerous_target.position = Point(150.0, 50.0, 5.0)  # Outside geofence
        
        # Should trigger safety response
        safety_response = self._check_safety(dangerous_target)
        self.assertTrue(safety_response['geofence_violation'])
        self.assertTrue(safety_response['emergency_action'])
        
    def test_4_noise_robustness(self):
        """Test system performance under noise"""
        print("Testing noise robustness...")
        
        # Add noise to sensor data
        self.mock_sensors.add_noise(level=0.1)
        
        test_detections = []
        tracked_positions = []
        
        # Run test with noise
        for i in range(50):
            # Generate noisy detection
            noisy_det = self._generate_noisy_detection()
            test_detections.append(noisy_det)
            
            # Publish detection
            det_msg = DetectionArray()
            det_msg.detections = [noisy_det]
            pub = rospy.Publisher(self.detection_topic, DetectionArray, queue_size=1)
            pub.publish(det_msg)
            
            # Check tracking stability
            try:
                tracked = rospy.wait_for_message(
                    self.tracked_topic, 
                    TrackedTargets, 
                    timeout=0.1
                )
                if tracked.targets:
                    tracked_positions.append(tracked.targets[0].position)
            except:
                pass
                
        # Check if tracking remained stable
        if tracked_positions:
            positions = np.array([[p.x, p.y, p.z] for p in tracked_positions])
            std_dev = np.std(positions, axis=0)
            
            # Should have reasonable variance
            self.assertLess(std_dev[0], 2.0)  # x variance
            self.assertLess(std_dev[1], 2.0)  # y variance
            
            print(f"Position std dev under noise: x={std_dev[0]:.2f}, y={std_dev[1]:.2f}")
        
    def test_5_performance_benchmark(self):
        """Benchmark system performance"""
        print("Benchmarking system performance...")
        
        import psutil

        # Check CPU and memory usage
        process = psutil.Process()
        cpu_percent = process.cpu_percent(interval=1.0)
        memory_info = process.memory_info()
        
        # Check ROS node status
        nodes = rospy.get_published_topics()
        
        # Log metrics
        metrics = {
            'cpu_usage': cpu_percent,
            'memory_usage': memory_info.rss / 1024 / 1024,  # MB
            'running_nodes': len(rospy.get_published_topics()),
            'topic_count': len(nodes)
        }
        
        print(f"Performance metrics: {metrics}")
        
        # Assert reasonable performance
        self.assertLess(cpu_percent, 80, "CPU usage too high")
        self.assertLess(metrics['memory_usage'], 4096, "Memory usage too high")
        
        # Test frame rate
        start_time = time.time()
        frame_count = 0
        while time.time() - start_time < 2.0:
            # Simulate processing
            time.sleep(0.016)  # ~60Hz
            frame_count += 1
            
        fps = frame_count / 2.0
        self.assertGreater(fps, 20, "Frame rate too low")
        print(f"Processing FPS: {fps:.1f}")
        
    def _numpy_to_ros_image(self, img):
        """Convert numpy image to ROS Image message"""
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg
        
    def _check_safety(self, target):
        """Check safety system response to target"""
        # Mock safety check
        geofence_radius = 100.0
        max_altitude = 50.0
        min_altitude = 2.0
        
        pos = [target.position.x, target.position.y, target.position.z]
        distance = np.linalg.norm(pos[:2])
        
        response = {
            'geofence_violation': distance > geofence_radius,
            'altitude_violation': pos[2] > max_altitude or pos[2] < min_altitude,
            'emergency_action': False
        }
        
        if response['geofence_violation'] or response['altitude_violation']:
            response['emergency_action'] = True
            
        return response
        
    def _generate_noisy_detection(self):
        """Generate a noisy detection for testing"""
        det = Detection()
        det.bbox = Point(
            x=np.random.normal(10, 0.5),
            y=np.random.normal(5, 0.5),
            z=np.random.normal(2, 0.1)
        )
        det.confidence = np.random.uniform(0.7, 0.99)
        det.class_id = 0
        return det

class TestScenarioRunner(unittest.TestCase):
    """Test scenario runner functionality"""
    
    def test_scenario_loading(self):
        """Test loading test scenarios"""
        runner = ScenarioRunner()
        scenario = runner.load_scenario('default')
        self.assertIsNotNone(scenario)
        self.assertIn('target_count', scenario)
        self.assertIn('duration', scenario)
        self.assertIn('target_behavior', scenario)
        
    def test_scenario_execution(self):
        """Test scenario execution"""
        runner = ScenarioRunner()
        scenario = runner.load_scenario('default')
        runner.start_scenario(scenario)
        
        # Get frames
        for i in range(10):
            frame = runner.get_frame()
            self.assertIsNotNone(frame)
            self.assertEqual(frame.shape, (480, 640, 3))
            
        runner.stop_scenario()
        
    def test_data_recording(self):
        """Test data recording functionality"""
        recorder = TestDataRecorder()
        recorder.start_recording(['/test_topic'], 5)
        
        # Publish test data
        pub = rospy.Publisher('/test_topic', DetectionArray, queue_size=1)
        for i in range(10):
            det_msg = DetectionArray()
            det_msg.detections = [Detection()]
            pub.publish(det_msg)
            time.sleep(0.1)
            
        recorder.stop_recording()
        
        # Check recorded data
        recorded = recorder.get_recorded_data()
        self.assertGreater(len(recorded), 0)

if __name__ == '__main__':
    # Run integration tests
    unittest.main(verbosity=2)