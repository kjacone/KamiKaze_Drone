# drone_control/msg/DetectionArray.msg
# Array of detections

std_msgs/Header header      # Message header
Detection[] detections      # Array of detections
int32 count                 # Number of detections
float32 processing_time     # Processing time in seconds
string source               # Detection source (camera, radar, etc.)