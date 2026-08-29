

string mission_id           # Unique mission identifier
string state                # Current mission state (idle, searching)
string previous_state       # Previous state
float32 elapsed_time        # Time in current state (seconds)
int32 current_waypoint      # Current waypoint index
int32 total_waypoints       # Total waypoints in mission
int32 target_count          # Number of tracked targets
float32 target_distance     # Distance to current target (meters)
int32 target_id             # Current target ID
string status_message       # Additional status information
time timestamp              # Message timestamp