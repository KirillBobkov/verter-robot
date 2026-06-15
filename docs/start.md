cd ~/verter-robot && git pull  
cd ~/ros2_ws && colcon build --packages-select verter_admin --symlink-install && source install/setup.bash
ros2 launch verter_admin chassis_bringup.launch.py
ros2 launch verter_admin mapping.launch.py
ros2 launch verter_admin waypoint_ui.launch.py

verter --nav /home/jetson/maps/map_20260227_142836.yaml

ros2 launch verter_admin autonomous_mapping_real.launch.py stop_distance:=0.15 resume_distance:=0.20

ros2 run verter_admin teleop_keyboard

ssh jetson@109.195.134.20 -p 3333
ssh jetson@192.168.0.9
ssh -X jetson@192.168.0.9 "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && rviz2 -d ~/verter-robot/verter_admin/verter_slam.rviz"

ssh -X jetson@192.168.0.10 "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && rviz2 -d ~/verter-robot/verter_admin/verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz

lsof /dev/esp32_chassis

cd ~/verter-robot/verter_admin
./diagnostics/monitor_esp32_links.sh

tail -n 80 dmesg.log
tail -n 120 service.log
tail -n 80 devices.log
tail -n 80 system.log
tail -n 80 tegrastats.log
