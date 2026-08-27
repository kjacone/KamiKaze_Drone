# drone_control/msg/Command.msg
# Command message for drone control

string command              # Command name (start, stop, pause, etc.)
string source               # Command source (user, script, emergency)
string data                 # JSON string with command data
int32 priority              # Command priority (0=critical, 3=low)
time timestamp              # Command timestamp