#!/usr/bin/env python3
"""
drone_control/test/test_mission_manager.py
Unit tests for mission manager
"""

import unittest

import rospy

from drone_control import Command, MissionStatus


class TestMissionManager(unittest.TestCase):
    """Test mission manager functionality"""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS node"""
        rospy.init_node('test_mission_manager', anonymous=True)
        cls.status_received = False
        cls.current_state = None
        
    def setUp(self):
        """Reset before each test"""
        self.status_received = False
        self.current_state = None
        
    def test_initialization(self):
        """Test mission manager initializes correctly"""
        # Check if mission status topic exists
        topics = rospy.get_published_topics()
        topic_names = [t[0] for t in topics]
        
        # Wait for mission status topic
        try:
            rospy.wait_for_message('/mission_status', MissionStatus, timeout=5.0)
            self.assertTrue(True, "Mission status topic found")
        except rospy.ROSException:
            self.fail("Mission status topic not found")
            
    def test_state_transition(self):
        """Test mission state transitions"""
        # Send start command
        cmd = Command()
        cmd.command = "start"
        cmd.source = "test"
        pub = rospy.Publisher('/drone_control/command', Command, queue_size=1)
        pub.publish(cmd)
        
        # Wait for status update
        try:
            status = rospy.wait_for_message('/mission_status', MissionStatus, timeout=5.0)
            self.assertIn(status.state, ['initializing', 'searching'])
        except rospy.ROSException:
            self.fail("No mission status received")

if __name__ == '__main__':
    unittest.main(verbosity=2)