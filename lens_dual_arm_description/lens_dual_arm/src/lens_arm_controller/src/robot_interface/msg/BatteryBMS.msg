float32 soc_percent # 电量百分比
float32 voltage_v # 电压
float32 current_a # 电流（负数为放电，正数为充电）
float32 temperature_c # 温度
int32 cycle_times # 循环次数
bool is_charging # 是否在充电
std_msgs/Header header # 时间戳