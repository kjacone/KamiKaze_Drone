# drone_control/msg/TrackedTargets.msg
# Array of tracked targets

std_msgs/Header header      # Message header
TrackedTarget[] targets     # Array of tracked targets
int32 count                 # Number of tracked targets