# ROS package for Amber B1
## What you will need
I assume that you have noetic & foxy & ros1_bridge on your PC 
## Steps to setting it up
After installing ros1_bridge and following some tutorials to get familiar with it. 
edit the static_bridge.cpp which is in (ros1_bridge_ws/src/ros1_bridge/src):
  std::string topic_name = "/position_trajectory_controller/joint_trajectory";
  std::string ros1_type_name = "trajectory_msgs/JointTrajectory";
  std::string ros2_type_name = "trajectory_msgs/msg/JointTrajectory";
  
here we are allowing only the /position_trajectory_controller/joint_trajectory topic to be bridged between noetic and foxy

rebuild the ros1_bridge and then source it 
## Steps to run it
On your amber b1 follow the normal procedure ( ros2_start.sh then b1.sh)
On your PC:
1) roslaunch amber_moveit_config demo.launch
2) ros2 run ros1_bridge static_bridge 
3) rosrun amber_linking_node amber_link.py 

Plan your move with moveit and press excute it should be excuted on your robot.:)
If you have questions reach me on Instagram Shannak_M
