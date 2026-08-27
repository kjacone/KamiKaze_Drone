# drone_control/msg/TrackedTarget.msg
# Tracked target information

int32 id                    # Target ID
geometry_msgs/Point position # Target position
geometry_msgs/Vector3 velocity # Target velocity
float32 confidence          # Tracking confidence (0-1)
string state                # Target state (tracking, engaging, etc.)
float32 distance            # Distance from drone
float32 heading             # Target heading angle
time timestamp              # Message timestamp