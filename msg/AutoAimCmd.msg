std_msgs/Header header
float64 pitch          # gimbal pitch angle (rad)
float64 yaw            # gimbal yaw angle (rad)
bool detected          # target detected
bool tracking          # target being tracked
bool fire              # fire command
float64 distance       # distance to target (m)
int32 proj_x           # reprojected pixel x
int32 proj_y           # reprojected pixel y
