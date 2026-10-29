# 时间戳和坐标系名称
std_msgs/Header header

# 本次扫描序号
uint32 scan_sequence

# 本次扫描圈数
uint8 frame_count

# 所有检测到的障碍物
radar_interfaces/Obstacle[] obstacles