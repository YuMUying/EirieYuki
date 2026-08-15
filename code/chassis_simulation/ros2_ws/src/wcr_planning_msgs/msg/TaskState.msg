uint8 IDLE=0
uint8 WAITING_DEVICES=1
uint8 EXECUTING=2
uint8 MANUAL_CONTROL=3
uint8 PAUSED_MANUAL=4
uint8 EMERGENCY_STOP=5
uint8 COMPLETED=6
uint8 FAILED=7
uint8 CANCELED=8

std_msgs/Header header
uint8 state
uint8 active_task_type
string task_id
string detail
bool manual_takeover
bool emergency_stop
bool autonomy_fault
