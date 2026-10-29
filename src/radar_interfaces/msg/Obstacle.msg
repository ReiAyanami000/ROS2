# 障碍中心在雷达场地坐标系中的坐标，单位：米
float64 x
float64 y

# 方格编号；255表示方格区域外
uint8 grid_id

# 检测统计
uint8 frame_hits
uint8 confidence_percent
uint16 point_count