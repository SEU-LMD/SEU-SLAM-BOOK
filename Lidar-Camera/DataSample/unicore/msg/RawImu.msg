# message 268
std_msgs/Header header

# gnss 周
uint32 week

# 周秒
float64 seconds_into_week

# IMU state: 详情见clap-B手册表1-16
uint32 imu_state

# z,y,x 轴速度变化, 需要乘比例因子变为速度，clap-B手册表1-14
float64 z_accel_output
float64 y_accel_output
float64 x_accel_output

# z,y,x 陀螺仪变化，需要乘比例因子变为角度,clap-B手册表1-14
float64 z_gyro_output
float64 y_gyro_output
float64 x_gyro_output