#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State

def state_cb(msg):
    rospy.loginfo(f"[vehicle_state_monitor] connected={msg.connected} armed={msg.armed} mode={msg.mode}")

def main():
    rospy.init_node("vehicle_state_monitor")
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.loginfo("vehicle_state_monitor node started")
    rospy.spin()

if __name__ == "__main__":
    main()