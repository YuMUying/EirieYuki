uint8 NONE=0
uint8 MOVE_TO_GOAL=1
uint8 FOLLOW_CURVE=2
uint8 SEAM_INSPECTION=3
uint8 CANCEL=4

std_msgs/Header header
uint8 task_type
string task_id
geometry_msgs/PoseStamped goal
nav_msgs/Path reference_curve
