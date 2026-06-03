#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_interface/srv/motion_execute.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#ifdef LENS_ENABLE_ACTUATOR_SDK
#include "actuator_controller.h"
#endif

enum class ControllerState {
  IDLE,
  EXECUTING_ACTION,
  RETURNING_TO_ZERO,
  WAITING_FOR_ZERO,
  FOLLOWING_EXTERNAL,
  HOLDING_EXTERNAL
};
enum class MotionCommand { PLAY_ACTION, RETURN_TO_ZERO, STOP_ACTION };

class JointControllerNode : public rclcpp::Node {
 public:
  JointControllerNode();

 private:
  // Service command dispatch.
  void handleCloudTask(
      const std::shared_ptr<robot_interface::srv::MotionExecute::Request>
          request,
      std::shared_ptr<robot_interface::srv::MotionExecute::Response> response);

  void handlePlayAction(
      const std::shared_ptr<robot_interface::srv::MotionExecute::Request>
          request,
      std::shared_ptr<robot_interface::srv::MotionExecute::Response> response);
  void handleReturnToZero(
      const std::shared_ptr<robot_interface::srv::MotionExecute::Request>
          request,
      std::shared_ptr<robot_interface::srv::MotionExecute::Response> response);
  void handleStopAction(
      const std::shared_ptr<robot_interface::srv::MotionExecute::Request>
          request,
      std::shared_ptr<robot_interface::srv::MotionExecute::Response> response);

  // Stored action playback.
  void executeActionDirectly(
      const std::shared_ptr<robot_interface::srv::MotionExecute::Request>
          request,
      std::shared_ptr<robot_interface::srv::MotionExecute::Response> response);
  void checkZeroAndStartAction();
  bool readData(std::string fileName);
  void publishJointCommand();

  // ROS topic bridge for simulator commands and hardware feedback.
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void externalJointCommandCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg);
  void checkExternalCommandTimeout();
  void publishExternalHold();
  void stopExternalHoldLocked();  // caller must hold mtx_
  sensor_msgs::msg::JointState normalizeExternalCommand(
      const sensor_msgs::msg::JointState& msg);
  bool checkAllJointsAtZero(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool validateJointCommand(const sensor_msgs::msg::JointState& msg,
                            std::string* error) const;
  void publishJointFeedback();

  // Return-to-zero motion.
  void startReturnToZero();
  void publishZeroPositionCommand();
  void stopReturnToZero();
  void publishExactZeroPositionCommand();

  // EtherCAT / MIT motor control.
  bool initEthercatController();
  bool ensureEthercatMotorsEnabled();
  void sendJointCommandToEthercat(const sensor_msgs::msg::JointState& msg);
  std::vector<std::vector<std::string>> armJointGroups() const;
  bool configureMotorGroupMitMode(const std::vector<std::string>& arm_joints,
                                  const char* arm_label, int retry_count,
                                  int retry_delay_ms);
  void loadInvertJointNamesParam();
  bool isInvertedJoint(const std::string& joint_name) const;
  double rosToMotorPosition(const std::string& joint_name, double q_ros) const;
  double motorToRosPosition(const std::string& joint_name, double q_motor) const;

  // Construction and small state-machine helpers.
  void declareParameters();
  void loadParameters();
  void configureJointLimits();
  void createRosInterfaces();
  void createTimers();
  void initializeLastExternalCommand();
  void enableEthercatIfConfigured();
  void setResponse(
      const std::shared_ptr<robot_interface::srv::MotionExecute::Response>& response,
      bool success, const std::string& message) const;
  bool canAcceptUserMotion() const;
  void preemptBlockingMotionForExternalLocked();
  void cancelPendingActionLocked();
  void resetToIdleLocked();
  std::string stateToString(ControllerState state);
  double smoothStep(double t);
  sensor_msgs::msg::JointState makeCommandMessage(
      const std::vector<std::string>& names,
      const std::vector<double>& positions) const;
  void publishAndSendCommand(const sensor_msgs::msg::JointState& msg);

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_command_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      external_joint_command_subscriber_;
  rclcpp::Service<robot_interface::srv::MotionExecute>::SharedPtr service_;
  std::vector<std::vector<double>> actionData_;
  uint32_t dataIndex_{0};
  uint32_t dataSize_{0};
  std::string path_;
  std::mutex mtx_;
  double zero_position_tolerance_{0.03};
  std::atomic<bool> all_at_zero_{false};
  std::atomic<bool> is_returning_to_zero_{false};
  ControllerState current_state_;
  rclcpp::TimerBase::SharedPtr action_timer_;
  rclcpp::TimerBase::SharedPtr return_to_zero_timer_;
  rclcpp::TimerBase::SharedPtr wait_for_zero_timer_;
  rclcpp::TimerBase::SharedPtr external_command_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr external_hold_timer_;
  rclcpp::TimerBase::SharedPtr joint_feedback_timer_;

  rclcpp::Time return_to_zero_start_time_;
  rclcpp::Time last_external_command_time_;
  sensor_msgs::msg::JointState last_external_command_;
  double return_to_zero_duration_{3.0};
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
  std::unordered_map<std::string, MotionCommand> motion_commands_;

  std::shared_ptr<robot_interface::srv::MotionExecute::Request>
      pending_action_request_;
  std::shared_ptr<robot_interface::srv::MotionExecute::Response>
      pending_action_response_;

  std::unordered_map<std::string, double> zero_positions_ = {
      {"Left_Shoulder_Pitch_Joint", 0.0}, {"Left_Shoulder_Roll_Joint", 0.0},
      {"Left_Shoulder_Yaw_Joint", 0.0},   {"Left_Elbow_Pitch_Joint", 0.0},
      {"Left_Wrist_Yaw_Joint", 0.0},      {"Left_Wrist_Roll_Joint", 0.0},
      {"Left_Wrist_Pitch_Joint", 0.0},    {"Right_Shoulder_Pitch_Joint", 0.0},
      {"Right_Shoulder_Roll_Joint", 0.0}, {"Right_Shoulder_Yaw_Joint", 0.0},
      {"Right_Elbow_Pitch_Joint", 0.0},   {"Right_Wrist_Yaw_Joint", 0.0},
      {"Right_Wrist_Roll_Joint", 0.0},    {"Right_Wrist_Pitch_Joint", 0.0},
  };

  std::vector<std::string> joint_name_ = {
      "Left_Shoulder_Pitch_Joint", "Left_Shoulder_Roll_Joint",
      "Left_Shoulder_Yaw_Joint",   "Left_Elbow_Pitch_Joint",
      "Left_Wrist_Yaw_Joint",      "Left_Wrist_Roll_Joint",
      "Left_Wrist_Pitch_Joint",    "Right_Shoulder_Pitch_Joint",
      "Right_Shoulder_Roll_Joint", "Right_Shoulder_Yaw_Joint",
      "Right_Elbow_Pitch_Joint",   "Right_Wrist_Yaw_Joint",
      "Right_Wrist_Roll_Joint",    "Right_Wrist_Pitch_Joint"};

  bool accept_external_joint_command_{true};
  std::string external_joint_command_topic_{"/joint_command"};
  double external_command_idle_timeout_s_{0.5};
  bool hold_last_command_on_idle_{true};
  double external_hold_publish_hz_{50.0};
  double joint_feedback_hz_{50.0};
  bool enforce_joint_position_limits_{true};
  std::unordered_map<std::string, std::pair<double, double>> joint_position_limits_;

  bool use_ethercat_control_{false};
  std::string ethercat_ifname_{"eth0"};
  std::string actuator_config_file_;
  std::string actuator_params_file_;
  int ethercat_send_frequency_{500};
  int motor_init_post_ec_delay_ms_{2000};
  int motor_init_retry_count_{5};
  int motor_init_retry_delay_ms_{400};
  double mit_kp_{10.0};
  double mit_kd_{1.0};
  double mit_vel_{0.0};
  double mit_torque_{0.0};
  std::unordered_set<std::string> invert_joint_names_;

  int consecutive_mit_failures_{0};

#ifdef LENS_ENABLE_ACTUATOR_SDK
  std::shared_ptr<ActuatorController> actuator_controller_;
#endif
};
