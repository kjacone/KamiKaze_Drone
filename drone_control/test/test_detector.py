#!/usr/bin/env python3
# drone_control/test/test_detector.py

import os
import unittest

import cv2
import numpy as np
import rospy
import torch
from drone_control import YOLODetector
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

from drone_control import Detection, DetectionArray


class TestYOLODetector(unittest.TestCase):
    """Unit tests for YOLO detector implementation"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS node and detector once for all tests"""
        rospy.init_node('test_detector', anonymous=True)
        cls.detector = YOLODetector()
        cls.test_image_path = os.path.join(
            os.path.dirname(__file__), 
            'test_data', 
            'sample_drone_view.jpg'
        )
        
    def setUp(self):
        """Reset state before each test"""
        self.detector.reset()
        
    def test_initialization(self):
        """Test detector initialization parameters"""
        self.assertIsNotNone(self.detector)
        self.assertIsNotNone(self.detector.model)
        self.assertEqual(self.detector.confidence_threshold, 0.5)
        self.assertEqual(self.detector.input_size, 416)
        
    def test_inference_shape(self):
        """Test that inference produces correct output shape"""
        # Load test image
        img = cv2.imread(self.test_image_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (416, 416))
        img = img.transpose(2, 0, 1) / 255.0
        img = torch.from_numpy(img).float().unsqueeze(0)
        
        # Run inference
        detections = self.detector.forward(img)
        
        # Check output structure
        self.assertIsInstance(detections, list)
        self.assertLessEqual(len(detections), 100)  # Max detections
        
        for det in detections:
            self.assertIn('bbox', det)
            self.assertIn('confidence', det)
            self.assertIn('class_id', det)
            self.assertEqual(len(det['bbox']), 4)
            self.assertGreaterEqual(det['confidence'], 0.0)
            self.assertLessEqual(det['confidence'], 1.0)
            
    def test_post_processing(self):
        """Test NMS and post-processing"""
        # Create test detections
        dummy_detections = [
            {'bbox': [10, 10, 50, 50], 'confidence': 0.9, 'class_id': 0},
            {'bbox': [12, 12, 52, 52], 'confidence': 0.8, 'class_id': 0},
            {'bbox': [100, 100, 30, 30], 'confidence': 0.7, 'class_id': 1}
        ]
        
        # Apply NMS
        processed = self.detector.post_process(dummy_detections, nms_threshold=0.5)
        
        # Check NMS worked
        self.assertEqual(len(processed), 2)  # Two overlapping boxes merged
        self.assertEqual(processed[0]['class_id'], 0)
        self.assertEqual(processed[1]['class_id'], 1)
        
    def test_frame_processing(self):
        """Test complete frame processing pipeline"""
        # Create dummy ROS Image message
        img_msg = Image()
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = 'bgr8'
        img_msg.step = img_msg.width * 3
        
        # Create dummy image data
        dummy_data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        img_msg.data = dummy_data.tobytes()
        
        # Process frame
        detection_array = self.detector.process_frame(img_msg)
        
        # Check output
        self.assertIsInstance(detection_array, DetectionArray)
        for detection in detection_array.detections:
            self.assertIsInstance(detection, Detection)
            self.assertTrue(0 <= detection.confidence <= 1.0)
            self.assertIsInstance(detection.bbox, Point)
            
    def test_edge_cases(self):
        """Test detector with edge cases"""
        # Empty frame
        empty_img = np.zeros((416, 416, 3), dtype=np.uint8)
        detections = self.detector.detect(empty_img)
        self.assertEqual(len(detections), 0)
        
        # Single pixel frame
        tiny_img = np.zeros((1, 1, 3), dtype=np.uint8)
        detections = self.detector.detect(tiny_img)
        self.assertEqual(len(detections), 0)
        
        # Invalid image
        with self.assertRaises(ValueError):
            self.detector.detect(None)
            
    def test_performance(self):
        """Test detector performance metrics"""
        import time
        img = cv2.imread(self.test_image_path)
        
        # Measure inference time
        start_time = time.time()
        for _ in range(10):
            detections = self.detector.detect(img)
        avg_time = (time.time() - start_time) / 10
        
        # Should be faster than 50ms for real-time
        self.assertLess(avg_time, 0.05)
        print(f"Average inference time: {avg_time:.3f}s")
        
        # Check FPS
        fps = 1.0 / avg_time
        self.assertGreater(fps, 20)
        print(f"FPS: {fps:.1f}")
        
    def test_model_accuracy(self):
        """Test detector accuracy on known dataset"""
        import json
        test_data_path = os.path.join(
            os.path.dirname(__file__),
            'test_data',
            'detection_test_data.json'
        )
        
        with open(test_data_path, 'r') as f:
            test_cases = json.load(f)
            
        total_cases = len(test_cases)
        correct_detections = 0
        total_detections = 0
        
        for test_case in test_cases:
            img = cv2.imread(test_case['image_path'])
            expected = test_case['expected_detections']
            
            detections = self.detector.detect(img)
            total_detections += len(detections)
            
            # Compare with expected (simple matching)
            matched = 0
            for det in detections:
                for exp in expected:
                    iou = self._compute_iou(det['bbox'], exp['bbox'])
                    if iou > 0.5 and det['class_id'] == exp['class_id']:
                        matched += 1
                        break
                        
            if matched >= len(expected) * 0.8:  # 80% match threshold
                correct_detections += 1
                
        accuracy = correct_detections / total_cases
        self.assertGreater(accuracy, 0.7)
        print(f"Detection accuracy: {accuracy:.2%}")
        
    def _compute_iou(self, bbox1, bbox2):
        """Compute Intersection over Union"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Calculate intersection
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
            
        intersection = (x_right - x_left) * (y_bottom - y_top)
        union = w1 * h1 + w2 * h2 - intersection
        
        return intersection / union

if __name__ == '__main__':
    # Run with coverage
    import coverage
    cov = coverage.Coverage(source=['../src/detector'])
    cov.start()
    
    unittest.main(verbosity=2, exit=False)
    
    cov.stop()
    cov.save()
    cov.report()
    cov.html_report(directory='coverage_html')