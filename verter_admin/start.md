cd ~/verter-robot && git pull                                                                                                                                                                                             
cd ~/ros2_ws && colcon build --packages-select verter_admin --symlink-install && source install/setup.bash                                                                                                                
ros2 launch verter_admin mapping.launch.py 

ros2 launch verter_admin autonomous_mapping_real.launch.py stop_distance:=0.20 resume_distance:=0.25

ros2 run verter_admin teleop_keyboard

ssh jetson@109.195.134.20 -p 3333
ssh jetson@192.168.0.21
ssh -X jetson@192.168.0.21 "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && rviz2 -d ~/verter-robot/verter_admin/verter_slam.rviz" 
