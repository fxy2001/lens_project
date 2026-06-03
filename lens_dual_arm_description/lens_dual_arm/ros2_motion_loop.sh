#!/bin/bash

# ROS2服务调用循环脚本
# 功能：每隔5分钟调用指定的ROS2服务
# 作者：编程助手
# 日期：2026

export HOME=/root
export LOG_FILE="/root/ros2_motion_service_loop.log"
export ROS_DOMAIN_ID=8
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="/dev/null"
sleep 3

# ===================== 配置项 =====================
# ROS2服务名称
SERVICE_NAME="/motion_controller/execute_motion"
# 服务消息类型
SERVICE_TYPE="robot_interface/srv/MotionExecute"
# 服务请求参数
SERVICE_ARGS="{motion_id: 'guojia', motion_name: ''}"
# 循环间隔（秒），150秒
INTERVAL=150
MAX_RUNS=30  # 执行30次退出
# ==================== 配置项结束 ====================

# 检查ROS2环境是否已配置
check_ros2_env() {
    #if ! command -v ros2 &> /dev/null; then
    #    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 错误：未找到ROS2环境，请先配置ROS2（source setup.bash）" | tee -a $LOG_FILE
    #    source /home/lens/lens_dual_arm/install/setup.bash
    #fi
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 配置ROS2（source setup.bash）" | tee -a $LOG_FILE
    #whoami
    source /opt/ros/humble/setup.bash
    source /home/lens/lens_dual_arm/install/setup.bash
}

# 执行ROS2服务调用
execute_service_call() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 开始调用ROS2服务: $SERVICE_NAME" | tee -a $LOG_FILE
    
    #response=$(ros2 service call $SERVICE_NAME $SERVICE_TYPE "{motion_id: 'stop', motion_name: ''}" 2>&1 < /dev/null)
    response=$(ros2 service call $SERVICE_NAME $SERVICE_TYPE "{motion_id: 'stop', motion_name: ''}" 2>&1)
    return_code=$?
    
    if [ $return_code -eq 0 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 服务调用成功！响应：$response" | tee -a $LOG_FILE
    else
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 服务调用失败！错误信息：$response" | tee -a $LOG_FILE
    fi
    sleep 2
    # 执行服务调用并捕获输出和返回码
    #response=$(ros2 service call $SERVICE_NAME $SERVICE_TYPE "$SERVICE_ARGS" 2>&1 < /dev/null)
    response=$(ros2 service call $SERVICE_NAME $SERVICE_TYPE "$SERVICE_ARGS" 2>&1)
    return_code=$?
    
    if [ $return_code -eq 0 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 服务调用成功！响应：$response" | tee -a $LOG_FILE
    else
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 服务调用失败！错误信息：$response" | tee -a $LOG_FILE
    fi
}

# 主循环函数
main_loop() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 启动ROS2服务循环调用脚本，间隔${INTERVAL}秒" | tee -a $LOG_FILE
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 服务名称：$SERVICE_NAME" | tee -a $LOG_FILE
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 日志文件：$LOG_FILE" | tee -a $LOG_FILE
    echo "================================================" | tee -a $LOG_FILE
    count=1
    # 无限循环执行
    while [ $count -le $MAX_RUNS ]; do
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 开始第 $count/$MAX_RUNS 次执行" | tee -a $LOG_FILE
        # 执行服务调用
        execute_service_call
        
        # 等待指定时间（5分钟）
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] 等待${INTERVAL}秒后再次执行..." | tee -a $LOG_FILE
        sleep $INTERVAL

	count=$((count + 1))
    done
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ✅ 已执行完成 $MAX_RUNS 次，自动退出脚本" | tee -a $LOG_FILE
}

# ==================== 脚本入口 ====================
# 1. 检查ROS2环境
check_ros2_env

# 2. 启动主循环
main_loop
