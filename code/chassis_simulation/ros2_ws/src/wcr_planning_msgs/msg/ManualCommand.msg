uint8 STOP=0
uint8 FORWARD=1
uint8 BACKWARD=2
uint8 STRAFE_LEFT=3
uint8 STRAFE_RIGHT=4
uint8 TURN_LEFT=5
uint8 TURN_RIGHT=6
uint8 ROTATE_LEFT=7
uint8 ROTATE_RIGHT=8

std_msgs/Header header
bool takeover
uint8 command
float64 linear_speed
float64 angular_speed
