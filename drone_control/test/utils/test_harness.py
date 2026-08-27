#!/usr/bin/env python3
"""
drone_control/test/utils/test_harness.py
Test harness for automated testing
"""

import rospy
import time
import unittest
import json
from typing import Dict, Any, Optional
from drone_control.msg import Command, CommandResponse
from std_msgs.msg import String

class TestHarness:
    """Test harness for automated testing"""
    
    def __init__(self, test_name: str = "test_harness"):
        self.test_name = test_name
        self.test_results = []
        self.current_test = None
        self.start_time = None
        
        # Publishers
        self.command_pub = rospy.Publisher('/drone_control/command', Command, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/drone_control/command_response', CommandResponse, self._response_callback)
        rospy.Subscriber('/system_status', String, self._status_callback)
        
        # State
        self.last_response = None
        self.last_status = None
        
    def _response_callback(self, msg):
        """Handle command responses"""
        self.last_response = msg
        
    def _status_callback(self, msg):
        """Handle status messages"""
        self.last_status = msg
        
    def send_command(self, command: str, data: Optional[Dict] = None, timeout: float = 5.0) -> bool:
        """Send a command and wait for response"""
        cmd_msg = Command()
        cmd_msg.command = command
        cmd_msg.source = self.test_name
        cmd_msg.data = json.dumps(data or {})
        cmd_msg.timestamp = rospy.Time.now()
        
        self.last_response = None
        self.command_pub.publish(cmd_msg)
        
        # Wait for response
        start_time = time.time()
        while self.last_response is None and time.time() - start_time < timeout:
            rospy.sleep(0.1)
            
        if self.last_response is None:
            rospy.logwarn(f"Command {command} timed out")
            return False
            
        return self.last_response.success
        
    def assert_command(self, command: str, data: Optional[Dict] = None, 
                       expected_success: bool = True, timeout: float = 5.0) -> bool:
        """Send command and assert response"""
        result = self.send_command(command, data, timeout)
        
        if result != expected_success:
            error_msg = f"Command {command} returned {result}, expected {expected_success}"
            if self.last_response:
                error_msg += f": {self.last_response.message}"
            raise AssertionError(error_msg)
            
        return result
        
    def assert_condition(self, condition: callable, timeout: float = 5.0, 
                         message: str = "Condition not met") -> bool:
        """Assert a condition becomes true within timeout"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if condition():
                return True
            rospy.sleep(0.1)
            
        raise AssertionError(message)
        
    def record_result(self, test_name: str, success: bool, message: str = ""):
        """Record test result"""
        self.test_results.append({
            'test_name': test_name,
            'success': success,
            'message': message,
            'timestamp': time.time()
        })
        
    def get_results(self) -> Dict:
        """Get test results summary"""
        total = len(self.test_results)
        passed = sum(1 for r in self.test_results if r['success'])
        failed = total - passed
        
        return {
            'test_harness': self.test_name,
            'total': total,
            'passed': passed,
            'failed': failed,
            'success_rate': passed / total if total > 0 else 0,
            'results': self.test_results
        }

class TestHarnessWrapper(unittest.TestCase):
    """Unittest wrapper for TestHarness"""
    
    def setUp(self):
        """Setup test harness"""
        self.harness = TestHarness(test_name=self.__class__.__name__)
        rospy.sleep(0.5)  # Allow initialization
        
    def tearDown(self):
        """Clean up test harness"""
        results = self.harness.get_results()
        if results['failed'] > 0:
            self.fail(f"Test harness failed: {results['failed']} failed out of {results['total']}")
            
    def test_command_status(self):
        """Test status command"""
        result = self.harness.assert_command('status', expected_success=True)
        self.assertTrue(result, "Status command failed")
        
    def test_command_invalid(self):
        """Test invalid command"""
        result = self.harness.assert_command('invalid_command_xyz', expected_success=False)
        self.assertFalse(result, "Invalid command should fail")

if __name__ == '__main__':
    unittest.main(verbosity=2)