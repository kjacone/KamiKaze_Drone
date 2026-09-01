#!/usr/bin/env python3
"""
drone_control/test/test_commands.py
Unit tests for command interpreter
"""

import unittest

import rospy

from drone_control import Command, CommandResponse


class TestCommandInterpreter(unittest.TestCase):
    """Test command interpreter functionality"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS node"""
        rospy.init_node('test_commands', anonymous=True)
        
    def test_command_handling(self):
        """Test basic command handling"""
        # Send test command
        cmd = Command()
        cmd.command = "status"
        cmd.source = "test"
        cmd.data = "{}"
        
        pub = rospy.Publisher('/drone_control/command', Command, queue_size=1)
        pub.publish(cmd)
        
        # Wait for response
        try:
            response = rospy.wait_for_message('/drone_control/command_response', CommandResponse, timeout=5.0)
            self.assertTrue(response.success)
            self.assertEqual(response.original_command, "status")
        except rospy.ROSException:
            self.fail("No command response received")
            
    def test_invalid_command(self):
        """Test invalid command handling"""
        cmd = Command()
        cmd.command = "invalid_command_xyz"
        cmd.source = "test"
        
        pub = rospy.Publisher('/drone_control/command', Command, queue_size=1)
        pub.publish(cmd)
        
        try:
            response = rospy.wait_for_message('/drone_control/command_response', CommandResponse, timeout=5.0)
            self.assertFalse(response.success)
            self.assertIn("Unknown", response.message)
        except rospy.ROSException:
            self.fail("No command response received")

if __name__ == '__main__':
    unittest.main(verbosity=2)