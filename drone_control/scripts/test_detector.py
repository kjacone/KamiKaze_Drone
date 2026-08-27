#!/usr/bin/env python3
import rospy
from drone_control.msg import DetectedObjects, Object, BBox

class TestDetector:
    def __init__(self):
        rospy.init_node('test_detector', anonymous=True)
        self.objects_pub = rospy.Publisher('/detected_objects', DetectedObjects, queue_size=10)
        self.rate = rospy.Rate(5)  # 5 Hz
        rospy.loginfo("Test Detector started - publishing fake detections")
        
    def run(self):
        while not rospy.is_shutdown():
            # Create fake detection
            det = DetectedObjects()
            det.header.stamp = rospy.Time.now()
            det.header.frame_id = 'camera'
            
            # Create fake object
            obj = Object()
            obj.class_id = 2  # car
            obj.class_name = "car"
            obj.confidence = 0.95
            
            # Fake bounding box
            bbox = BBox()
            bbox.x = 100
            bbox.y = 100
            bbox.width = 50
            bbox.height = 50
            obj.bbox = bbox
            
            det.objects.append(obj)
            self.objects_pub.publish(det)
            
            rospy.loginfo_once("Published fake detection")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        detector = TestDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
