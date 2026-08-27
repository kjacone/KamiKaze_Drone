# drone_control/msg/ControlCommand.msg
# Control command for drone movement

string command_type          # position, velocity, attitude, thrust
string frame_id              # Reference frame (body, world, map)
geometry_msgs/Pose pose      # Target pose (position + orientation)
geometry_msgs/Twist twist    # Target velocity
float32 thrust               # Thrust value (0-1)
float32 yaw                  # Yaw angle (radians)
float32 yaw_rate             # Yaw rate (rad/s)
bool is_relative             # Whether command is relative to current position
int32 priority               # Command priority (0=highest)
time timestamp               # Command timestamp