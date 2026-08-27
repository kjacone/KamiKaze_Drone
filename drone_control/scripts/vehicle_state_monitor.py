#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from std_srvs.srv import Trigger, TriggerResponse

class DroneStateMonitor:
    def __init__(self):
        rospy.init_node('vehicle_state_monitor', anonymous=True)
        self.state = None
        self.armed = False
        self.mode = "UNKNOWN"
        self.safety_violations = []

        # Subscribers
        rospy.Subscriber('/mavros/state', State, self.state_callback)

        # Services
        rospy.Service('/drone_emergency_stop', Trigger, self.emergency_stop)
        rospy.Service('/drone_arm', Trigger, self.arm)
        rospy.Service('/drone_disarm', Trigger, self.disarm)

        # Publisher for status (optional)
        self.status_pub = rospy.Publisher('/drone_status', String, queue_size=10)

        rospy.loginfo("Vehicle State Monitor started")

    def state_callback(self, msg):
        self.state = msg
        self.armed = msg.armed
        self.mode = msg.mode
        # Log state changes
        rospy.loginfo_throttle(5, f"State: armed={msg.armed}, mode={msg.mode}, connected={msg.connected}")
        # Safety checks
        if not msg.connected:
            self.log_safety_violation("LOST_CONNECTION")
        # Publish status
        self.publish_status()

    def log_safety_violation(self, violation):
        if violation not in self.safety_violations:
            self.safety_violations.append(violation)
            rospy.logwarn(f"Safety violation: {violation}")

    def publish_status(self):
        # Could publish a custom status message
        pass

    def emergency_stop(self, req):
        rospy.logwarn("Emergency stop requested!")
        # Here you would send a land command or disarm
        # For now, just log
        return TriggerResponse(success=True, message="Emergency stop triggered")

    def arm(self, req):
        # Would call arming service
        rospy.loginfo("Arming requested (simulated)")
        return TriggerResponse(success=True, message="Arming command sent")

    def disarm(self, req):
        rospy.loginfo("Disarming requested (simulated)")
        return TriggerResponse(success=True, message="Disarming command sent")

if __name__ == '__main__':
    try:
        monitor = DroneStateMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass