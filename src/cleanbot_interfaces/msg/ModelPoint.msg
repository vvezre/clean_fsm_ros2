# 文件作用：定义 RTK 采集的建模点及其质量、角色和局部坐标消息。
string id
uint32 sequence
float64 lat
float64 lon
float64 x_cm
float64 y_cm
float64 heading_deg
string capture_type
string role
string[] roles
uint32 sample_count
float64 sample_radius_m
uint8 rtk_quality
float64 gga_age_sec
string source
