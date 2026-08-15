uint8 CIRCLE=0
uint8 RECTANGLE=1

string id
uint8 shape
geometry_msgs/Point center

# Circle size. Ignored for rectangles.
float64 radius

# Rectangle full dimensions. Ignored for circles.
float64 width
float64 height

# Rectangle rotation around its center in the odom plane, in radians.
float64 yaw
