192.168.0.19

cd squashfs-root
./AppRun


./AppRun --no-sandbox

通讯

```
# 控制模式：先保留 profile_position（更稳）
export LENS_CAN_CTRL_MODE=profile_position

# 提高轨迹跟随速度（现在偏低）
export LENS_PT_V=20.2
export LENS_PT_A=10.0
export LENS_PT_D=10.0

# 提高发送频率
export LENS_BRIDGE_HZ=200

# 放宽每关节限速（之前太小会明显拖手）
export LENS_MIT_MAX_POS_RATE_RAD_S=20.0

# 降低链路保守参数（减少每帧等待）
export LENS_CAN_FRAME_GAP_S=0.0001
export LENS_CAN_SEND_RETRY=0

# 初始化只做一次
export LENS_INIT_REPEAT_S=0

python -u left_arm_joint_slider_7d.py
```
逆解

```
cd /root/Desktop/lens_dual_arm_description/lens_dual_arm_description
source /opt/ros/humble/setup.bash

export LENS_BRIDGE_PROTOCOL=can
export LENS_CAN_BACKEND=hobot_hal
export LENS_HOBOT_CAN_CONFIG_ROOT=/app/Can/can_multi_ch/config
export LENS_HOBOT_CAN_TARGET=canglotx_ins0ch5
export LENS_HOBOT_CAN_CHANNEL=5
export LENS_HOBOT_CAN_PREFER_BYPASS=1
export LENS_CAN_SEND=1
export LENS_LEFT_CAN_IDS=1,2,3,4,5,6,7
unset LENS_ACTIVE_CAN_IDS

# 推荐先用你当前稳定的轨迹位置模式
export LENS_CAN_CTRL_MODE=profile_position
export LENS_PT_V=20.2
export LENS_PT_A=10.0
export LENS_PT_D=10.0
export LENS_INIT_REPEAT_S=0

export LENS_BRIDGE_HZ=200

# 放宽每关节限速（之前太小会明显拖手）
export LENS_MIT_MAX_POS_RATE_RAD_S=20.0

# 降低链路保守参数（减少每帧等待）
export LENS_CAN_FRAME_GAP_S=0.0001
export LENS_CAN_SEND_RETRY=0



python -u left_arm_ik_slider.py
```
逆解优化
```
cd /root/Desktop/lens_dual_arm_description/lens_dual_arm_description
source /opt/ros/humble/setup.bash

export LENS_BRIDGE_PROTOCOL=can
export LENS_CAN_BACKEND=hobot_hal
export LENS_HOBOT_CAN_CONFIG_ROOT=/app/Can/can_multi_ch/config
export LENS_HOBOT_CAN_TARGET=canglotx_ins0ch5
export LENS_HOBOT_CAN_CHANNEL=5
export LENS_CAN_SEND=1
export LENS_LEFT_CAN_IDS=1,2,3,4,5,6,7
unset LENS_ACTIVE_CAN_IDS
export LENS_HOBOT_CAN_PREFER_BYPASS=1  #
export LENS_CPU_SET=1-5

# 轨迹位置模式（更顺滑，且你前面验证过有效）
export LENS_CAN_CTRL_MODE=profile_position
export LENS_PT_V=15.2
export LENS_PT_A=5.0
export LENS_PT_D=5.0
export LENS_INIT_REPEAT_S=0

export LENS_BRIDGE_HZ=200
# 多核优化
export LENS_IK_WORKERS=5

# 解算/发送频率（可按需调快）
export LENS_IK_SOLVE_HZ=200
export LENS_IK_PUBLISH_HZ=200
export LENS_IK_TARGET_SMOOTH=0.30
export LENS_IK_MAX_JOINT_SPEED=2.5

# 下发链路（降低抖动）
export LENS_CAN_FRAME_GAP_S=0.0001
export LENS_CAN_SEND_RETRY=0
export LENS_INIT_REPEAT_S=0

python -u left_arm_ik_slider_headless_can_slider.py
```
逆解优化
```
cd /root/Desktop/lens_dual_arm_description/lens_dual_arm_description
source /opt/ros/humble/setup.bash

export LENS_BRIDGE_PROTOCOL=can
export LENS_CAN_BACKEND=hobot_hal
export LENS_HOBOT_CAN_CONFIG_ROOT=/app/Can/can_multi_ch/config
export LENS_HOBOT_CAN_TARGET=canglotx_ins0ch5
export LENS_HOBOT_CAN_CHANNEL=5
export LENS_HOBOT_CAN_PREFER_BYPASS=1
export LENS_CAN_SEND=1
export LENS_LEFT_CAN_IDS=1,2,3,4,5,6,7
unset LENS_ACTIVE_CAN_IDS

export LENS_CPU_SET=1-5
export LENS_IK_WORKERS=5

export LENS_CAN_CTRL_MODE=profile_position
export LENS_PT_V=10.0
export LENS_PT_A=8.0
export LENS_PT_D=8.0

export LENS_IK_SOLVE_HZ=180
export LENS_IK_PUBLISH_HZ=200
export LENS_IK_TARGET_SMOOTH=1.0
export LENS_IK_MAX_JOINT_SPEED=20.0

export LENS_CAN_FRAME_GAP_S=0.0001
export LENS_CAN_SEND_RETRY=0
export LENS_INIT_REPEAT_S=0

export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

python -u left_arm_ik_slider_headless_can_slider.py
```



## VLA
```
# 1) 先激活虚拟环境
source /root/Desktop/lens_dual_arm_description/lens_dual_arm_description/.venv/bin/activate

# 2) 关闭旧服务（占用8001）
kill 23967
# 或更稳妥：
pkill -f "llm_demo/fastapi_path_ir_headless_server.py" || true

# 3) 启新服务（可固定 8001）
cd /root/Desktop/lens_dual_arm_description
source /opt/ros/humble/setup.bash

export LENS_ROS_BRIDGE=1
export LENS_ROS_TOPIC=/joint_command
export LENS_ROS_HZ=200
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LENS_PATH_IR_PLAN_CACHE_DIR="$HOME/.lens_path_ir_plan_cache"
export LENS_FASTAPI_PORT=8001

python3 -u llm_demo/fastapi_path_ir_sim_server.py
```
