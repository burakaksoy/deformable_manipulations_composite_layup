#!/bin/bash
sleep 1s;

# gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; killall gzclient && killall gzserver; roscore; exec bash\"";
# sleep 1s;

gnome-terminal --tab --title="All" --command "bash -c \"source ~/.bashrc; 
                                                        roslaunch deformable_manipulations_composite_layup main_launcher_two_robot.launch is_remote:=true launch_controller:=false real_robot_mode_enabled:=false; 
                                                        exec bash\"";

# gnome-terminal --tab --title="All" --command "bash -c \"source ~/.bashrc; 
#                                                         roslaunch deformable_manipulations_composite_layup main_launcher_two_robot.launch is_remote:=true launch_controller:=false real_robot_mode_enabled:=true; 
#                                                         exec bash\"";

sleep 4s;

gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_deformable/src/deformable_manipulations_composite_layup/rviz/two_robot_composite_layup.rviz; exec bash\"";
# gnome-terminal --tab --title="RVIZ" --command "bash -c \"source ~/.bashrc; rosrun rviz rviz -d ~/catkin_ws_deformable/src/deformable_manipulations_composite_layup/rviz/two_robot_composite_layup_real.rviz; exec bash\"";
sleep 4s;

gnome-terminal --tab --title="GUI" --command "bash -c \"source ~/.bashrc; rosrun fabric_simulator test_gui.py _mode:="composite_sheet_application_test"; exec bash\"";
sleep 1s;

# To publish to "/space_nav/twist"
# # If you have a space mouse, you can use the following command to start the spacenav node:
# gnome-terminal --tab --title="Spacenav" --command "bash -c \"source ~/.bashrc; roslaunch spacenav_node classic.launch; exec bash\"";
# Otherwise, you can use rqt_ez_publisher:
gnome-terminal --tab --title="RQT_EZ_PUBLSHER" --command "bash -c \"source ~/.bashrc; rosrun rqt_ez_publisher rqt_ez_publisher; exec bash\"";
sleep 1s;

# gnome-terminal --tab --title="Controller" --command "bash -c \"source ~/.bashrc; roslaunch deformable_manipulations_composite_layup velocity_controller.launch; exec bash\"";
# sleep 1s;

# To start the controller, call the service with command:
# rosservice call /composite_layup_velocity_controller/set_enable_controller "data: true" 