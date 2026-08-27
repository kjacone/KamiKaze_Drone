#!/usr/bin/env python3
# drone_control/test/utils/mock_sensors.py

import numpy as np
import cv2
import random
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist
from drone_control.msg import Detection, DetectionArray

@dataclass
class Target:
    """Represents a simulated target"""
    id: int
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    size: np.ndarray      # [width, height, depth]
    color: Tuple[int, int, int]  # RGB
    trajectory_type: str  # 'stationary', 'linear', 'circular'
    trajectory_params: dict
    confidence: float = 0.95

class MockSensors:
    """Simulates sensor data for testing"""
    
    def __init__(self):
        self.targets = []
        self.next_id = 1
        self.noise_level = 0.0
        self.frame_counter = 0
        
        # Camera parameters
        self.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ])
        self.distortion_coeffs = np.zeros(5)
        self.image_width = 640
        self.image_height = 480
        
        # Sensor parameters
        self.sensor_noise = {
            'position': 0.1,
            'velocity': 0.05,
            'detection': 0.02,
            'gyro': 0.01,
            'accelerometer': 0.01
        }
        
    def add_noise(self, level: float):
        """Add noise to sensor readings"""
        self.noise_level = level
        for key in self.sensor_noise:
            self.sensor_noise[key] *= (1 + level)
            
    def create_target(self, position: np.ndarray, trajectory_type: str = 'stationary',
                     velocity: Optional[np.ndarray] = None, size: np.ndarray = np.array([1, 1, 1])):
        """Create a new simulated target"""
        target = Target(
            id=self.next_id,
            position=position,
            velocity=velocity if velocity is not None else np.zeros(3),
            size=size,
            color=(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)),
            trajectory_type=trajectory_type,
            trajectory_params={}
        )
        self.next_id += 1
        self.targets.append(target)
        return target
        
    def update_targets(self, dt: float):
        """Update target positions based on trajectories"""
        for target in self.targets:
            if target.trajectory_type == 'stationary':
                # Do nothing
                pass
            elif target.trajectory_type == 'linear':
                target.position += target.velocity * dt
            elif target.trajectory_type == 'circular':
                # Circular motion
                center = np.array(target.trajectory_params.get('center', [0, 0, 0]))
                radius = target.trajectory_params.get('radius', 5.0)
                speed = target.trajectory_params.get('speed', 1.0)
                
                angle = self.frame_counter * speed * dt
                target.position = center + np.array([
                    radius * np.cos(angle),
                    radius * np.sin(angle),
                    0
                ])
                target.velocity = np.array([
                    -radius * speed * np.sin(angle),
                    radius * speed * np.cos(angle),
                    0
                ])
            elif target.trajectory_type == 'random':
                # Random walk
                noise = np.random.normal(0, 0.1, 3)
                target.position += noise * dt
                
        self.frame_counter += 1
        
    def generate_camera_image(self) -> np.ndarray:
        """Generate simulated camera image with targets"""
        # Create background
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        img[:, :] = [100, 120, 140]  # Gray-blue background
        
        # Add ground grid
        self._add_grid(img)
        
        # Render targets
        for target in self.targets:
            # Project 3D position to 2D
            point_2d = self._project_3d_to_2d(target.position)
            
            if point_2d is not None:
                # Draw target
                size_pixels = int(20 * target.size[0] / np.linalg.norm(target.position))
                cv2.circle(img, point_2d, size_pixels, target.color, -1)
                
                # Add label
                label = f"ID:{target.id}"
                cv2.putText(img, label, (point_2d[0] + 10, point_2d[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Add trajectory indicator
                if target.trajectory_type != 'stationary':
                    end_point = self._project_3d_to_2d(
                        target.position + target.velocity * 0.5
                    )
                    if end_point is not None:
                        cv2.arrowedLine(img, tuple(point_2d), tuple(end_point),
                                       (255, 255, 0), 2)
        
        return img
        
    def _add_grid(self, img):
        """Add ground grid to image"""
        for i in range(0, self.image_width, 50):
            cv2.line(img, (i, 0), (i, self.image_height), (50, 60, 70), 1)
        for i in range(0, self.image_height, 50):
            cv2.line(img, (0, i), (self.image_width, i), (50, 60, 70), 1)
            
    def _project_3d_to_2d(self, point_3d: np.ndarray) -> Optional[Tuple[int, int]]:
        """Project 3D point to 2D image coordinates"""
        # Simple perspective projection
        if point_3d[2] <= 0:
            return None
            
        fov = 60  # degrees
        focal_length = self.image_width / (2 * np.tan(np.radians(fov / 2)))
        
        x_2d = int((point_3d[0] / point_3d[2]) * focal_length + self.image_width / 2)
        y_2d = int(-(point_3d[1] / point_3d[2]) * focal_length + self.image_height / 2)
        
        if 0 <= x_2d < self.image_width and 0 <= y_2d < self.image_height:
            return (x_2d, y_2d)
        return None
        
    def get_detections(self) -> DetectionArray:
        """Generate detections from current target positions"""
        det_msg = DetectionArray()
        
        for target in self.targets:
            # Add noise to position
            pos_noise = np.random.normal(0, self.sensor_noise['position'], 3)
            pos = target.position + pos_noise
            
            # Project to image
            point_2d = self._project_3d_to_2d(pos)
            if point_2d is not None:
                detection = Detection()
                detection.bbox = Point(
                    x=float(point_2d[0]),
                    y=float(point_2d[1]),
                    z=float(pos[2])
                )
                detection.confidence = max(0.1, min(0.99, 
                    target.confidence + np.random.normal(0, self.sensor_noise['detection'])
                ))
                detection.class_id = target.id
                detection.track_id = target.id
                det_msg.detections.append(detection)
                
        return det_msg
        
    def get_imu_data(self) -> dict:
        """Generate simulated IMU data"""
        return {
            'accelerometer': np.random.normal(0, self.sensor_noise['accelerometer'], 3),
            'gyroscope': np.random.normal(0, self.sensor_noise['gyro'], 3),
            'magnetometer': np.random.normal(0, 0.01, 3),
            'timestamp': rospy.Time.now()
        }
        
    def get_pose_data(self) -> Pose:
        """Generate simulated drone pose"""
        pose = Pose()
        
        # Add position noise
        pos_noise = np.random.normal(0, self.sensor_noise['position'], 3)
        pose.position.x = self.drone_position[0] + pos_noise[0]
        pose.position.y = self.drone_position[1] + pos_noise[1]
        pose.position.z = self.drone_position[2] + pos_noise[2]
        
        # Simple orientation
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = np.sin(self.frame_counter * 0.01)
        pose.orientation.w = np.cos(self.frame_counter * 0.01)
        
        return pose
        
    def set_drone_position(self, position: np.ndarray):
        """Set drone position for simulation"""
        self.drone_position = position

class ScenarioRunner:
    """Run test scenarios with mock data"""
    
    def __init__(self):
        self.sensor = MockSensors()
        self.current_scenario = None
        self.start_time = None
        self.is_running = False
        
    def load_scenario(self, scenario_name: str) -> dict:
        """Load a test scenario configuration"""
        import yaml
        import os
        
        # Load from config file
        config_path = os.path.join(
            os.path.dirname(__file__), 
            '..', 
            'config', 
            'test_scenarios',
            f'{scenario_name}.yaml'
        )
        
        with open(config_path, 'r') as f:
            scenario = yaml.safe_load(f)
            
        return scenario['scenario']
        
    def start_scenario(self, scenario: dict):
        """Start executing a test scenario"""
        self.current_scenario = scenario
        self.start_time = rospy.Time.now()
        self.is_running = True
        
        # Create targets based on scenario
        for i, target_config in enumerate(scenario['target_behavior']):
            position = np.array(target_config.get('position', [0, 0, 5]))
            trajectory_type = target_config['type']
            
            self.sensor.create_target(
                position=position,
                trajectory_type=trajectory_type,
                velocity=np.array(target_config.get('velocity', [0, 0, 0])),
                size=np.array(target_config.get('size', [1, 1, 1]))
            )
            
    def get_frame(self) -> np.ndarray:
        """Get next frame from scenario"""
        if not self.is_running:
            raise RuntimeError("Scenario not running")
            
        dt = 0.033  # 30 FPS
        self.sensor.update_targets(dt)
        
        # Check scenario duration
        if (rospy.Time.now() - self.start_time).to_sec() > self.current_scenario['duration']:
            self.is_running = False
            
        return self.sensor.generate_camera_image()
        
    def stop_scenario(self):
        """Stop the current scenario"""
        self.is_running = False
        self.current_scenario = None
        
    def get_target_positions(self) -> List[np.ndarray]:
        """Get current positions of all targets"""
        return [target.position for target in self.sensor.targets]

class TestDataRecorder:
    """Record test data for playback and analysis"""
    
    def __init__(self, output_dir: str = None):
        self.recording = False
        self.topics = []
        self.data = {}
        self.start_time = None
        
        if output_dir is None:
            import os
            self.output_dir = os.path.join(
                os.path.dirname(__file__),
                '..',
                'test_results',
                'recordings'
            )
        else:
            self.output_dir = output_dir
            
        os.makedirs(self.output_dir, exist_ok=True)
        
    def start_recording(self, topics: List[str], duration: float = None):
        """Start recording specified topics"""
        self.recording = True
        self.topics = topics
        self.data = {topic: [] for topic in topics}
        self.start_time = rospy.Time.now()
        self.duration = duration
        
        # Subscribe to topics
        self.subscribers = []
        for topic in topics:
            self.subscribers.append(
                rospy.Subscriber(topic, rospy.AnyMsg, self._callback, topic)
            )
            
    def _callback(self, msg, topic_name):
        """Callback for recording messages"""
        if self.recording:
            self.data[topic_name].append({
                'timestamp': rospy.Time.now(),
                'data': msg
            })
            
    def stop_recording(self):
        """Stop recording and save data"""
        self.recording = False
        
        # Save to file
        import pickle
        timestamp = rospy.Time.now().to_sec()
        filename = f"recording_{timestamp}.pkl"
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'wb') as f:
            pickle.dump(self.data, f)
            
        print(f"Recording saved to {filepath}")
        
    def get_recorded_data(self) -> dict:
        """Get recorded data as dictionary"""
        return self.data
        
    def playback(self, topic: str):
        """Playback recorded data on topic"""
        if topic not in self.data:
            raise ValueError(f"No data for topic {topic}")
            
        pub = rospy.Publisher(topic, rospy.AnyMsg, queue_size=10)
        
        for entry in self.data[topic]:
            timestamp = entry['timestamp']
            msg = entry['data']
            
            # Wait until correct time
            wait_time = (timestamp - self.start_time).to_sec()
            if wait_time > 0:
                rospy.sleep(wait_time)
                
            pub.publish(msg)
            
    def clear(self):
        """Clear recorded data"""
        self.data = {topic: [] for topic in self.topics}