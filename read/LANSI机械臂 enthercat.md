ethercat 通讯
```
sudo pkill -f joint_controller_node 2>/dev/null || true
sudo pkill -f joint_controller_executable 2>/dev/null || true

cd /home/ylgy/lens_ec_ws
sudo -E bash -lc '
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/ylgy/lens_ec_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm/third_party_runtime_libs:$LD_LIBRARY_PATH
export ROS_LOG_DIR=/tmp/ros_log; mkdir -p /tmp/ros_log

ros2 run joint_controller_node joint_controller_executable \
  --ros-args \
  --params-file /home/ylgy/lens_ec_ws/install/joint_controller_node/share/joint_controller_node/config/ethercat_control.yaml \
  -p ethercat_ifname:=enp12s0
'
```
注意地址和网卡根据部署情况修改

3D仿真桥接
```

source /opt/ros/jazzy/setup.bash; \
source /home/ylgy/lens_ec_ws/install/setup.bash; \
source ~/venv_ros_mujoco/bin/activate; \
export ROS_DOMAIN_ID=0; \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
export LENS_ROS_BRIDGE=1; \
export LENS_ROS_TOPIC=/joint_command; \
export LENS_ROS_HZ=20; \
export LENS_MIT_KP=20; \
export LENS_MIT_KD=8; \
python /home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm_description/left_arm_slider_trajectory.py

```
6D仿真桥接
```
source /opt/ros/jazzy/setup.bash
source /home/ylgy/lens_ec_ws/install/setup.bash
source ~/venv_ros_mujoco/bin/activate

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_command
export LENS_ROS_HZ=20

python /home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm_description/left_arm_ik_slider.py
```
右臂
```
python /home/ylgy/ 桌面/lens_dual_arm_description/lens_dual_arm_description/right_arm_ik_slider.py

```

双臂
```
source /opt/ros/jazzy/setup.bash
source /home/ylgy/lens_ec_ws/install/setup.bash
source ~/venv_ros_mujoco/bin/activate

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_command
export LENS_ROS_HZ=20


python /home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm_description/dual_arm_ik_slider.py
```


# S100
enthercat通讯

```
sudo pkill -f joint_controller_node 2>/dev/null || true
sudo pkill -f joint_controller_executable 2>/dev/null || true
sudo pkill -f topic_tools 2>/dev/null || true

cd /root/Desktop/lens_dual_arm_description/lens_dual_arm
sudo -E bash -lc '
source /opt/ros/humble/setup.bash
source /root/Desktop/lens_dual_arm_description/lens_dual_arm/install/setup.bash
export ROS_DOMAIN_ID=0
unset RMW_IMPLEMENTATION

ros2 run joint_controller_node joint_controller_executable \
  --ros-args \
  --params-file /root/Desktop/lens_dual_arm_description/ethercat_control.yaml \
  -p ethercat_ifname:=eth1 \
  -p actuator_config_file:=/root/Desktop/lens_dual_arm_description/actuator_SDK_ARM/actuator_config.json \
  -p actuator_params_file:=/root/Desktop/lens_dual_arm_description/actuator_SDK_ARM/actuator_params_config.json
'
```
双臂
```
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
unset RMW_IMPLEMENTATION
python /root/Desktop/lens_dual_arm_description/lens_dual_arm_description/dual_arm_joint_slider_ros.py
```
无仿真双臂
```
# 终端2：6D IK 无渲染发布（不需要 relay）
cd /root/Desktop/lens_dual_arm_description/lens_dual_arm_description
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
unset RMW_IMPLEMENTATION
export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_states
export LENS_ROS_HZ=100
export LENS_IK_LOOP_HZ=150
python -u dual_arm_ik_slider_headless_ros.py
```

逆解发布解耦（双臂加速）
```
cd /root/Desktop/lens_dual_arm_description/lens_dual_arm_description
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
unset RMW_IMPLEMENTATION

export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_states
export LENS_ROS_HZ=120

export LENS_IK_LOOP_HZ=220
export LENS_IK_SOLVE_HZ=90
export LENS_IK_MAX_ITERS=24
export LENS_IK_TARGET_SMOOTH=0.30
export LENS_IK_MAX_JOINT_SPEED=2.0

python -u dual_arm_ik_slider_headless_ros.py
```

ROS通讯控制节点示例
```
cd /root/Desktop/lens_dual_arm_description/lens_dual_arm_description
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
unset RMW_IMPLEMENTATION
python -u dual_arm_ik_command_node.py
```

```
source /opt/ros/humble/setup.bash
ros2 topic pub /dual_arm/command_6d std_msgs/msg/Float64MultiArray \
"{data: [0.30, 0.20, 1.00, 0.0, 0.0, 0.0, 0.30, -0.20, 1.00, 0.0, 0.0, 0.0]}" -r 30
```

```
source /opt/ros/humble/setup.bash
ros2 topic pub /dual_arm/left_pose_cmd geometry_msgs/msg/PoseStamped \
"{pose: {position: {x: 0.30, y: 0.20, z: 1.00}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}" -r 30
```

## 轨迹绘制

轨迹绘制双臂
```
source /opt/ros/jazzy/setup.bash
source ~/venv_ros_mujoco/bin/activate

export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_command
export LENS_ROS_HZ=120
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LENS_PATH_IR_PLAN_CACHE_DIR="$HOME/.lens_path_ir_plan_cache"

python3 /home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm_description/llm_demo/fastapi_path_ir_sim_server.py
```

本地json
```
curl -X POST "http://127.0.0.1:8000/trajectory"   -H "Content-Type: application/json"   --data-binary "@/home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm_description/llm_demo/ir.json"
```

通讯
```
sudo pkill -f joint_controller_node 2>/dev/null || true
sudo pkill -f joint_controller_executable 2>/dev/null || true

cd /home/ylgy/lens_ec_ws
sudo -E bash -lc '
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source /home/ylgy/lens_ec_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/ylgy/桌面/lens_dual_arm_description/lens_dual_arm/third_party_runtime_libs:$LD_LIBRARY_PATH
export ROS_LOG_DIR=/tmp/ros_log; mkdir -p /tmp/ros_log

ros2 run joint_controller_node joint_controller_executable \
  --ros-args \
  --params-file /home/ylgy/lens_ec_ws/install/joint_controller_node/share/joint_controller_node/config/ethercat_control.yaml \
  -p ethercat_ifname:=enp12s0 \
  -p mit_kp:=40.0 \
  -p mit_kd:=8.0
'
```