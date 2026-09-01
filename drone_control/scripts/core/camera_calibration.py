
import geometry_msgs.msg
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo

from drone_control.msg import NodeHealth


class CameraCalibration:
    def __init__(self):
        rospy.init_node('camera_calibration', anonymous=True)
        self.pub_info = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # Static transform from drone base to camera
        self.tf_msg = geometry_msgs.msg.TransformStamped()
        self.tf_msg.header.frame_id = 'base_link'
        self.tf_msg.child_frame_id = 'camera_link'
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.1
        self.tf_msg.transform.rotation.w = 1.0
        # Camera info (from YAML)
        self.camera_info = CameraInfo()
        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.K = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        self.camera_info.P = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.loginfo("Camera calibration node started")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Publish static transform
            self.tf_msg.header.stamp = rospy.Time.now()
            self.tf_broadcaster.sendTransform(self.tf_msg)
            # Publish camera info
            self.camera_info.header.stamp = rospy.Time.now()
            self.camera_info.header.frame_id = 'camera_link'
            self.pub_info.publish(self.camera_info)
            rate.sleep()

if __name__ == '__main__':
    node = CameraCalibration()
    node.run()