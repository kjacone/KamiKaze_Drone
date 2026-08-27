# drone_control/msg/NodeHealth.msg
# Node health status

string node_name            # Node name
string status               # Node status (running, error, stopped, etc.)
bool is_healthy             # Overall health status
int32 detection_count       # Number of detections (for detector nodes)
int32 error_count           # Number of errors encountered
float32 fps                 # Frames per second (for detector nodes)
float32 cpu_usage           # CPU usage percentage
float32 memory_usage        # Memory usage percentage
time timestamp              # Message timestamp