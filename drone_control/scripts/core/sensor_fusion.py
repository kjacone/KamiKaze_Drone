#!/usr/bin/env python3
"""
drone_control/scripts/core/sensor_fusion.py
Sensor fusion node
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
from drone_control.msg import NodeHealth

class SensorFusion:
    """Sensor fusion node"""
    
    def __init__(self):
        # Initialize ROS node FIRST
        rospy.init_node('sensor_fusion', anonymous=False)
        
        self.publish_rate = rospy.get_param('~publish_rate', 20)
        self.simulation_mode = rospy.get_param('/use_simulation', True)
        self.test_mode = rospy.get_param('/test_mode', False)
        
        # State
        self.imu_data = None
        self.gps_data = None
        self.odom_data = None
        
        # Subscribers
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # Publishers
        self.health_pub = rospy.Publisher('/sensor_fusion/node_health', NodeHealth, queue_size=10)
        
        # Timer
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self.publish_health)
        
        rospy.loginfo("Sensor Fusion initialized")
        
    def imu_callback(self, msg):
        self.imu_data = msg
        
    def gps_callback(self, msg):
        self.gps_data = msg
        
    def pose_callback(self, msg):
        self.odom_data = msg
        
    def publish_health(self, event):
        health_msg = NodeHealth()
        health_msg.node_name = 'sensor_fusion'
        health_msg.status = 'running'
        health_msg.timestamp = rospy.Time.now()
        health_msg.is_healthy = True
        self.health_pub.publish(health_msg)

if __name__ == '__main__':
    try:
        fusion = SensorFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass