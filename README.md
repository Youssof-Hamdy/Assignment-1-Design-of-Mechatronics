Youssof Hamdy Sayed Amed		9473
Moaz Yasser				9746

1. SYSTEM INITIALIZATION
cd ~/lab1_ws
colcon build
source install/setup.bash
ros2 launch turtle_control turtlesim_control_launch.py


2. OPEN NEW TERMINAL
source install/setup.bash

2.1 DRAW SQUARE 
ros2 action send_goal /draw_square my_interface/action/GoToPose "{}"

2.2 GOING TO SPECIFIC COORDINATE
ros2 action send_goal /go_to_pose my_interface/action/GoToPose "{target: {x: 2.0, y: 8.0, yaw: 1.57}}"

2.3 SETTING PARAMETERS
ros2 service call /set_speed_limits my_interface/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.0}"

2.4 RESET 
ros2 service call /trigger_clean_reset my_interface/srv/SetSpeedLimits "{}"



3. GRAPHICAL OVERVIEW
ros2 run rqt_graph rqt_graph


4. RECORDING

4.1 START RECORDING
ros2 bag record /turtle1/cmd_vel /turtle1/pose

4.2 STOP RECORDING
ctrl+c

4.3 OPEN NEW TERMINAL
4.4 PLAY RECORDING
ros2 bag play rosbag2_2026_*

*== time stamp of recording can be found in main workspace folder




