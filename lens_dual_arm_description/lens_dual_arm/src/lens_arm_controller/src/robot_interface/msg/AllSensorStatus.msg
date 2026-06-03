# robot_interface/msg/AllSensorStatus.msg
SensorStatus[] statuses  # 传感器状态数组
std_msgs/Header header               # 时间戳
string frame_id          # 坐标系标识
