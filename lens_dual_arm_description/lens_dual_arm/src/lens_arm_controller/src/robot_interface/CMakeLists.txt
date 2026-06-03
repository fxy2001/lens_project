cmake_minimum_required(VERSION 3.5)
project(robot_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_dir "msg")
set(msg_files
  ${msg_dir}/RobotState.msg
  ${msg_dir}/RobotTaskInfo.msg
  ${msg_dir}/NavStatus.msg
  ${msg_dir}/SensorStatus.msg
  ${msg_dir}/AllSensorStatus.msg
  ${msg_dir}/BatteryBMS.msg
  ${msg_dir}/CurrentMap.msg
  ${msg_dir}/CurrentTask.msg
  ${msg_dir}/EStopStatus.msg
  ${msg_dir}/NavigationStatus.msg
  ${msg_dir}/RobotPoseInMap.msg
)

set(srv_dir "srv")
set(srv_files
  ${srv_dir}/SetInt16.srv
  ${srv_dir}/GetInt16.srv
  ${srv_dir}/MoveTo.srv
  ${srv_dir}/NavToPose.srv
  ${srv_dir}/GetFloat32MultiArray.srv
  ${srv_dir}/SetUint32.srv
  ${srv_dir}/SetRobotTask.srv
  ${srv_dir}/ExecuteCommand.srv
  ${srv_dir}/MotionExecute.srv
)

set(action_dir "action")
set(action_files
  ${action_dir}/ChassisCommand.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()
