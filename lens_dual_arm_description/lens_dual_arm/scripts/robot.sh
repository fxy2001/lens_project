#!/bin/bash
###
 # @Description:   robot.sh
 # @Version: V1.0
 # @Author: zw_1520@163.com
 # @Date: 2025-08-27 09:28:53
 # @LastEditors: zw_1520@163.com
 # @LastEditTime: 2025-09-08 16:39:06
 # Copyright (C) 2024-2050 Lens All rights reserved.
### 
project_path=${ROBOT_DIR}
echo "progect_path: ${project_path}"

LOG_DIR=${project_path}/logs
LOG_FILE=${LOG_DIR}/shell_1.log

function rotate_logs() {
	log_index=5
	while [ ${log_index} -gt 0 ]; do
		if [ -f shell_${log_index}.log ]; then
			log_info "rotate shell_${log_index}.log"
			let target_log_index=log_index+1
			mv ${LOG_DIR}/shell_${log_index}.log ${LOG_DIR}/shell_${target_log_index}.log
		fi
		let log_index=log_index-1
	done
}

function log_info() {
	DATE_N=$(date "+%Y-%m-%d %H:%M:%S")
	USER_N=$(whoami)
	LOG_MSG="${DATE_N} ${USER_N} [INFO] $@"
	echo "${LOG_MSG}"
	echo "${LOG_MSG}" >>$LOG_FILE
}

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

echo "creat soft link"
ln -snf ${LOG_DIR}/${current_day}/${current_date} ${project_path}/latest

NUMBER_OFFSET=2 #delete two days ago log file
echo "days offset to save log files: "${NUMBER_OFFSET}

ls ${LOG_DIR} |
  while read line; do
    # Skip non-date entries
    if ! [[ $line =~ ^[0-9]{4}$ ]]; then
      continue
    fi
    
    echo "line is " + ${line}
    tmp=$((10#${line} + NUMBER_OFFSET))
    echo "temp is " + ${tmp}
    if [ ${tmp} -gt ${current_day} ]; then
      echo "${tmp} > > > ${current_day}"
      echo "recently two day's log file, should retain "
    else
      echo "${tmp} <<<< ${current_day}"
      echo "warn: [ old logs, should remove ]"
      rm -rf ${LOG_DIR}/${line}/
    fi
  done

##################end for logs setting##############
echo "$@: " $@

cd_to() {
  cd $@
}


######  copied from bashrc ###########
export ROS_DOMAIN_ID=1
source /opt/ros/jazzy/setup.bash
source /home/robot/ti5robot_ws/install/setup.bash

#################

#rotate_logs


#node 1: robot grpc server
log_info "ros2 launch robot_grpc_server robot_grpc_server_launch.py > ${LOG_DIR}/${current_day}/${current_date}/robot_grpc_server.log 2>&1"
${UN_BUF} ros2 launch robot_grpc_server robot_grpc_server_launch.py >${LOG_DIR}/${current_day}/${current_date}/robot_grpc_server.log 2>&1 &

#node 2: robot camera control
log_info "ros2 run camera_control camera_control_node > ${LOG_DIR}/${current_day}/${current_date}/robot_grpc_server.log 2>&1"
${UN_BUF} ros2 run camera_control camera_control_node >${LOG_DIR}/${current_day}/${current_date}/camera_control.log 2>&1 &

#node 3: robot tts player
log_info "ros2 run lens_ros2_node subscriber_node > ${LOG_DIR}/${current_day}/${current_date}/lens_tts_subscriber.log 2>&1"
${UN_BUF} ros2 run lens_ros2_node subscriber_node >${LOG_DIR}/${current_day}/${current_date}/lens_tts_subscriber.log 2>&1 &

i=0
while true; do
  let i=i+1
  log_info "i is $i"
  sleep 30
  if [[ $i -gt 10000 ]]; then
    i=0
  fi
done
