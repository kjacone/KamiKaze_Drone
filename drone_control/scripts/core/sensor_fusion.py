
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State

class SensorFusion:
    def __init__(self):
        rospy.init_node('sensor_fusion', anonymous=True)
        self.pose_pub = rospy.Publisher('/fused_pose', PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/mavros/local_position/odometry', Odometry, self.odom_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.latest_pose = None
        rospy.loginfo("Sensor fusion node started")

    def odom_callback(self, msg):
        # Forward odometry as fused pose (can later fuse with vision)
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.pose_pub.publish(pose_msg)
        self.latest_pose = pose_msg

    def state_callback(self, msg):
        # Log state for debugging
        pass

if __name__ == '__main__':
    rospy.spin()