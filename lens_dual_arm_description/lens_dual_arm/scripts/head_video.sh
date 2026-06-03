#!/bin/bash
###
 # @Description:   robot.sh
 # @Version: V1.0
 # @Author: zw_1520@163.com
 # @Date: 2025-08-27 09:28:53
 # @LastEditors: zw_1520@163.com
 # @LastEditTime: 2025-08-29 09:34:07
 # Copyright (C) 2024-2050 Lens All rights reserved.
### 
project_path=${ROBOT_DIR}
echo "progect_path: ${project_path}"

LOG_DIR=${project_path}/logs
LOG_FILE=${LOG_DIR}/shell_1.log


####set for robot logs##############
echo "log_path: ${LOG_DIR}"

if [ -d ${LOG_DIR} ]; then
  echo "${LOG_DIR} directory exists"
else
  echo "${LOG_DIR} does not exists, so let's  mkdir it"
  mkdir -p ${LOG_DIR}
fi

current_day=$(date +%m%d)
echo "current day: "${current_day}
current_date=$(date +%H.%M.%S)
echo "current date: "${current_date}

echo "mkdir directory about day and time"

mkdir -p ${LOG_DIR}/${current_day}/${current_date}/diagnostics_log/


target_ip=10.15.229.248
duration=180  # 3分钟=180秒
start_time=$(date +%s)

echo "开始持续检测 $target_ip 的连通性,持续3分钟..."

used_time=0
while [ $(( $(date +%s) - start_time )) -lt $duration ]; do
    # 执行ping检查
    ping -c 3 $target_ip > /dev/null 2>&1
    
    # 检查ping结果
    if [ $? -eq 0 ]; then
        echo "$(date '+%Y-%m-%d %H:%M:%S') - $target_ip 可达"
        break
    else
        echo "$(date '+%Y-%m-%d %H:%M:%S') - $target_ip 不可达"
    fi
    
    sleep 1  # 每秒检测一次
    ${used_time} = ${used_time}+1
done

echo "检测结束, 总耗时 ${used_time} 秒."

#node 1: robot grpc server
/home/robot/rtsp_code/gst-rtsp-server/builddir/examples/test-uri -i rtsp://admin:123456@192.168.0.250:554/Streaming/Channels/101 >${LOG_DIR}/${current_day}/${current_date}/head_rtsp_video_service.log 2>&1 &

i=0
while true; do
  let i=i+1
  log_info "i is $i"
  sleep 30
  if [[ $i -gt 10000 ]]; then
    i=0
  fi
done
