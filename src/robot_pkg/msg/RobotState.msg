Header header

# Whether or not the robot has triggered a software kill
bool killed

# Position on table, in meters
float64 pos_x
float64 pos_y

# Planar velocity, in meters/second
float64 vel_x
float64 vel_y

# Current heading, in radians
float64 yaw

# Orientation of robot, specifically for slope
# Maybe replace with 3D "gravity vector" or something?
float64 pitch
float64 roll

# Rate of yaw change, in radians/second
float64 angular_vel

# Boolean of whether robot is at target (if false, currently moving)
bool at_target
