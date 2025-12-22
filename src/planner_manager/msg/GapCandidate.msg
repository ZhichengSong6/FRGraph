# representative direction 
geometry_msgs/Vector3 dir

# yaw and elev bias for limited gap
float32 yaw_bias
float32 elev_bias

# spherical angles (rad) and mean range (m)
float32 center_yaw
float32 center_elev
float32 range_mean

# angular spans (rad) within the subregion
float32 yaw_span
float32 elev_span

# pixel count in the subregion (for confidence/weighting)
int32 size

# 0: OPEN, 1: LIMITED, 2: FREE
uint8 type