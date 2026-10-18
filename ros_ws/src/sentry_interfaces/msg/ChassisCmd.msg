# ID of task
uint8 tid

# Chassis control modes
uint8 VELOCITY = 1
uint8 FOLLOW_GIMBAL = 2
uint8 SWING = 3
uint8 SPIN = 4

uint8 type 1
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel
