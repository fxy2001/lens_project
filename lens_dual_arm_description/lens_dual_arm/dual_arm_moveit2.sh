#export ROS_DOMAIN_ID=52
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lens_dual_arm_moveit_config lens_dual_arm_moveit.launch.py
