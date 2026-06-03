cmake_minimum_required(VERSION 3.8)
project(joint_controller_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interface REQUIRED)

add_executable(joint_controller_executable src/main.cpp
                                          src/joint_controller.cpp)

option(ENABLE_ACTUATOR_SDK "Enable real EtherCAT actuator SDK" OFF)
set(ACTUATOR_SDK_DIR "" CACHE PATH "Path to actuator_SDK_X86 directory")
set(ACTUATOR_SDK_LIBRARY "" CACHE FILEPATH "Path to actuator SDK shared library")
set(SOEM_LIBRARY "" CACHE FILEPATH "Path to libsoem static/shared library")

if(ENABLE_ACTUATOR_SDK)
  if(NOT ACTUATOR_SDK_DIR)
    message(FATAL_ERROR "ENABLE_ACTUATOR_SDK=ON requires ACTUATOR_SDK_DIR")
  endif()
  if(NOT ACTUATOR_SDK_LIBRARY)
    set(ACTUATOR_SDK_LIBRARY "${ACTUATOR_SDK_DIR}/libactuator_controller.a")
  endif()
  if(NOT SOEM_LIBRARY)
    set(SOEM_LIBRARY "${ACTUATOR_SDK_DIR}/libsoem.a")
  endif()
  if(NOT EXISTS "${ACTUATOR_SDK_LIBRARY}")
    message(FATAL_ERROR "ACTUATOR_SDK_LIBRARY not found: ${ACTUATOR_SDK_LIBRARY}")
  endif()
  if(NOT EXISTS "${SOEM_LIBRARY}")
    message(FATAL_ERROR "SOEM_LIBRARY not found: ${SOEM_LIBRARY}")
  endif()
  target_compile_definitions(joint_controller_executable PRIVATE LENS_ENABLE_ACTUATOR_SDK=1)
  target_include_directories(joint_controller_executable PRIVATE ${ACTUATOR_SDK_DIR})
  # The actuator_SDK static library is built against specific fmt/spdlog versions.
  # When the host's fmt/spdlog differs, you can get undefined references at link time.
  # Prefer the pinned runtime libs we ship under `third_party_runtime_libs`.
  set(WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
  set(LENS_THIRD_PARTY_RUNTIME_LIB_DIR "${WS_ROOT}/third_party_runtime_libs" CACHE PATH "Pinned fmt/spdlog runtime libs")
  set(LENS_THIRD_PARTY_FMT_SO "${LENS_THIRD_PARTY_RUNTIME_LIB_DIR}/libfmt.so.8.1.1")
  set(LENS_THIRD_PARTY_SPDLOG_A "${LENS_THIRD_PARTY_RUNTIME_LIB_DIR}/libspdlog.a")

  set(_use_pinned_third_party FALSE)
  if(EXISTS "${LENS_THIRD_PARTY_FMT_SO}" AND EXISTS "${LENS_THIRD_PARTY_SPDLOG_A}")
    execute_process(
      COMMAND file -b "${LENS_THIRD_PARTY_FMT_SO}"
      OUTPUT_VARIABLE _fmt_file_info
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(_fmt_file_info MATCHES "x86-64" AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
      message(STATUS "joint_controller_node: skip pinned fmt/spdlog (x86 lib on ${CMAKE_SYSTEM_PROCESSOR})")
    elseif(_fmt_file_info MATCHES "ARM" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
      message(STATUS "joint_controller_node: skip pinned fmt/spdlog (ARM lib on ${CMAKE_SYSTEM_PROCESSOR})")
    else()
      set(_use_pinned_third_party TRUE)
    endif()
  endif()

  if(_use_pinned_third_party)
    message(STATUS "joint_controller_node: linking pinned fmt/spdlog runtime libs")
    target_link_libraries(
      joint_controller_executable
      ${ACTUATOR_SDK_LIBRARY}
      ${SOEM_LIBRARY}
      "${LENS_THIRD_PARTY_SPDLOG_A}"
      "${LENS_THIRD_PARTY_FMT_SO}"
      pthread
      dl
      rt
    )
  else()
    find_package(spdlog REQUIRED)
    find_package(fmt REQUIRED)
    target_link_libraries(
      joint_controller_executable
      ${ACTUATOR_SDK_LIBRARY}
      ${SOEM_LIBRARY}
      spdlog::spdlog
      fmt::fmt
      pthread
      dl
      rt
    )
  endif()
  message(STATUS "joint_controller_executable: EtherCAT SDK enabled")
else()
  message(WARNING "joint_controller_executable: EtherCAT SDK disabled; topic-only mode")
endif()

ament_target_dependencies(joint_controller_executable
  rclcpp
  sensor_msgs
  std_msgs
  robot_interface
)

install(TARGETS
  joint_controller_executable
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(joint_controller_executable)

ament_package()