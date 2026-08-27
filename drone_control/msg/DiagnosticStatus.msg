# drone_control/msg/DiagnosticStatus.msg
# System diagnostic status

time timestamp              # Message timestamp
string system_health        # JSON string with system health data
string node_status          # JSON string with node status
string performance_metrics  # JSON string with performance metrics
string safety_status        # JSON string with safety status
string mission_status       # JSON string with mission status