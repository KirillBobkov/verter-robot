# Leishen ROS 2 driver

Source: https://github.com/Lslidar/Lslidar_ROS2_driver/tree/M10P/N10P
Commit: 8bb760c8f66c1b6964caaf3cf6f7a2a0ea24cefa
Packages: lslidar_driver, lslidar_msgs. Keep upstream copyright and GPL notices.
Vendored here so tools/build.sh builds the pinned driver together with Verter.
Robot configuration: verter_bringup/config/lidar/m10p.yaml.

Local patch: correct single-return LaserScan angle_increment and time_increment for the inclusive endpoint array. The original reported an angle_max one bin before the actual last ray. Verified with M10P UART at 500000 baud (~12 Hz).
