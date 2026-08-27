# drone_control/msg/CommandResponse.msg
# Response to a command

string original_command     # Original command that was sent
bool success                # Whether command succeeded
string message              # Response message
string data                 # JSON string with response data
time timestamp              # Response timestamp