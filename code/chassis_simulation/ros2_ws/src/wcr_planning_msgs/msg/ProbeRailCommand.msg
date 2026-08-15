uint8 DISABLE=0
uint8 HOME=1
uint8 POSITION=2
uint8 HOLD=3
uint8 STOP=4
uint8 TRACK_SEAM=5

std_msgs/Header header
uint8 mode
float64 target_position_m
float64 max_velocity_m_s
float64 max_acceleration_m_s2
string source
