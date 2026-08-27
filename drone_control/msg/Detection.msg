# drone_control/msg/Detection.msg
# Single detection message

int32 class_id              # Object class ID
string class_name           # Object class name
float32 confidence          # Detection confidence (0-1)
geometry_msgs/Point center  # Detection center in image coordinates
float32 width               # Bounding box width
float32 height              # Bounding box height
float32 depth               # Estimated depth/distance
float32 x                   # Center x in pixels
float32 y                   # Center y in pixels
int32 track_id              # Track ID if associated with a track
time timestamp              # Detection timestamp