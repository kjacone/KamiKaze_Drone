#!/usr/bin/env python3
import rospy
import sys

def main():
    rospy.init_node('test_recorder', anonymous=True)
    rospy.loginfo("Test Recorder started (dummy)")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass