# drone_control/msg/SafetyStatus.msg
# Safety status message

bool is_safe                # Overall safety status
string[] violations         # List of safety violations
string emergency_reason     # Reason for emergency if active
bool geofence_enabled       # Whether geofence is active
bool emergency_active       # Whether emergency mode is active
float32 geofence_radius     # Geofence radius in meters
float32 current_distance    # Current distance from origin
float32 current_altitude    # Current altitude
time timestamp              # Message timestamp