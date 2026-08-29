#!/usr/bin/env python3
"""
drone_control/scripts/test_verification.py
Test verification node for automated testing
"""

import rospy
import time
import json
from drone_control.msg import TrackedTargets, SafetyStatus, MissionStatus
from std_msgs.msg import String

class TestVerification:
    """Verify test results against expected outputs"""
    
    def __init__(self):
        rospy.init_node('test_verification', anonymous=False)
        
        self.scenario = rospy.get_param('~scenario', 'default')
        self.tolerance = rospy.get_param('~tolerance', 0.1)
        self.expected_outputs = rospy.get_param('~expected_outputs', True)
        
        # State
        self.tracking_data = []
        self.safety_data = []
        self.mission_data = []
        self.start_time = time.time()
        
        # Subscribers - FIXED with correct message types
        rospy.Subscriber('/tracked_targets', TrackedTargets, self._tracking_callback)
        rospy.Subscriber('/safety_status', SafetyStatus, self._safety_callback)
        rospy.Subscriber('/mission_status', MissionStatus, self._mission_callback)
        
        # Publishers
        self.results_pub = rospy.Publisher('/test_results', String, queue_size=10)
        
        rospy.loginfo(f"Test verification started for scenario: {self.scenario}")
        
    def _tracking_callback(self, msg):
        """Collect tracking data"""
        self.tracking_data.append({
            'timestamp': time.time(),
            'count': msg.count,
            'targets': [{'id': t.id, 'confidence': t.confidence, 'distance': t.distance} 
                       for t in msg.targets]
        })
        
    def _safety_callback(self, msg):
        """Collect safety data - FIXED for SafetyStatus fields"""
        self.safety_data.append({
            'timestamp': time.time(),
            'is_safe': msg.is_safe,
            'violations': list(msg.violations),
            'emergency_active': msg.emergency_active,
            'emergency_reason': msg.emergency_reason
        })
        
    def _mission_callback(self, msg):
        """Collect mission data"""
        self.mission_data.append({
            'timestamp': time.time(),
            'state': msg.state,
            'elapsed_time': msg.elapsed_time
        })
        
    def verify_results(self):
        """Verify collected results against expectations"""
        results = {
            'scenario': self.scenario,
            'duration': time.time() - self.start_time,
            'tracking_received': len(self.tracking_data) > 0,
            'safety_received': len(self.safety_data) > 0,
            'mission_received': len(self.mission_data) > 0,
            'tracking_count': len(self.tracking_data),
            'safety_count': len(self.safety_data),
            'mission_count': len(self.mission_data)
        }
        
        # Check if tracking data is valid
        if self.tracking_data:
            max_targets = max([d['count'] for d in self.tracking_data])
            results['max_targets'] = max_targets
            
        # Check if safety violations occurred
        if self.safety_data:
            violations = [d for d in self.safety_data if not d['is_safe']]
            results['safety_violations'] = len(violations)
            results['emergency_events'] = sum([1 for d in self.safety_data if d['emergency_active']])
            
        # Check if mission completed
        if self.mission_data:
            final_state = self.mission_data[-1]['state']
            results['final_state'] = final_state
            
        # Publish results
        results_msg = String()
        results_msg.data = json.dumps(results)
        self.results_pub.publish(results_msg)
        
        rospy.loginfo(f"Test results: {results}")
        return results

if __name__ == '__main__':
    try:
        verifier = TestVerification()
        rospy.sleep(5)  # Wait for data collection
        
        # Verify results
        results = verifier.verify_results()
        
        rospy.loginfo("Test verification complete")
        
    except rospy.ROSInterruptException:
        pass