#include "joint_controller.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace {

std::string trimCopy(const std::string& s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
    --e;
  }
  return s.substr(b, e - b);
}

}  // namespace

JointControllerNode::JointControllerNode() : Node("lens_arm_controller_node") {
  std::filesystem::path current_file(__FILE__);
  path_ = std::string(current_file.parent_path().parent_path().parent_path()) +
          "/action_data/";
  RCLCPP_INFO(this->get_logger(), "Action data directory: %s", path_.c_str());

  declareParameters();
  loadParameters();
  configureJointLimits();
  createRosInterfaces();
  createTimers();

  current_state_ = ControllerState::IDLE;
  return_to_zero_duration_ = 3.0;

  motion_commands_["guojia"] = MotionCommand::PLAY_ACTION;
  motion_commands_["return_zero"] = MotionCommand::RETURN_TO_ZERO;
  motion_commands_["stop"] = MotionCommand::STOP_ACTION;
  motion_commands_["resume_external"] = MotionCommand::STOP_ACTION;

  initializeLastExternalCommand();
  enableEthercatIfConfigured();
}

void JointControllerNode::declareParameters() {
  this->declare_parameter<bool>("use_ethercat_control", false);
  this->declare_parameter<std::string>("ethercat_ifname", "eth0");
  this->declare_parameter<std::string>("actuator_config_file", "");
  this->declare_parameter<std::string>("actuator_params_file", "");
  this->declare_parameter<int>("ethercat_send_frequency", 500);
  this->declare_parameter<int>("motor_init_post_ec_delay_ms", 2000);
  this->declare_parameter<int>("motor_init_retry_count", 5);
  this->declare_parameter<int>("motor_init_retry_delay_ms", 400);
  this->declare_parameter<double>("mit_kp", 10.0);
  this->declare_parameter<double>("mit_kd", 1.0);
  this->declare_parameter<double>("mit_vel", 0.0);
  this->declare_parameter<double>("mit_torque", 0.0);
  this->declare_parameter<bool>("accept_external_joint_command", true);
  this->declare_parameter<std::string>("external_joint_command_topic",
                                       "/joint_command");
  this->declare_parameter<double>("external_command_idle_timeout_s", 0.5);
  this->declare_parameter<bool>("hold_last_command_on_idle", true);
  this->declare_parameter<double>("external_hold_publish_hz", 50.0);
  this->declare_parameter<double>("joint_feedback_hz", 50.0);
  this->declare_parameter<bool>("enforce_joint_position_limits", true);
  this->declare_parameter<std::string>("invert_joint_names", "");
}

void JointControllerNode::loadParameters() {
  use_ethercat_control_ = this->get_parameter("use_ethercat_control").as_bool();
  ethercat_ifname_ = this->get_parameter("ethercat_ifname").as_string();
  actuator_config_file_ =
      this->get_parameter("actuator_config_file").as_string();
  actuator_params_file_ =
      this->get_parameter("actuator_params_file").as_string();
  ethercat_send_frequency_ =
      this->get_parameter("ethercat_send_frequency").as_int();
  motor_init_post_ec_delay_ms_ =
      this->get_parameter("motor_init_post_ec_delay_ms").as_int();
  motor_init_retry_count_ =
      this->get_parameter("motor_init_retry_count").as_int();
  motor_init_retry_delay_ms_ =
      this->get_parameter("motor_init_retry_delay_ms").as_int();
  mit_kp_ = this->get_parameter("mit_kp").as_double();
  mit_kd_ = this->get_parameter("mit_kd").as_double();
  mit_vel_ = this->get_parameter("mit_vel").as_double();
  mit_torque_ = this->get_parameter("mit_torque").as_double();
  accept_external_joint_command_ =
      this->get_parameter("accept_external_joint_command").as_bool();
  external_joint_command_topic_ =
      this->get_parameter("external_joint_command_topic").as_string();
  external_command_idle_timeout_s_ =
      this->get_parameter("external_command_idle_timeout_s").as_double();
  hold_last_command_on_idle_ =
      this->get_parameter("hold_last_command_on_idle").as_bool();
  external_hold_publish_hz_ =
      this->get_parameter("external_hold_publish_hz").as_double();
  joint_feedback_hz_ = this->get_parameter("joint_feedback_hz").as_double();
  enforce_joint_position_limits_ =
      this->get_parameter("enforce_joint_position_limits").as_bool();
  loadInvertJointNamesParam();
}

void JointControllerNode::configureJointLimits() {
  joint_position_limits_ = {
      {"Left_Shoulder_Pitch_Joint", {-3.14, 3.14}},
      {"Left_Shoulder_Roll_Joint", {-0.3, 3.2}},
      {"Left_Shoulder_Yaw_Joint", {-3.14, 3.14}},
      {"Left_Elbow_Pitch_Joint", {-1.8, 0.2}},
      {"Left_Wrist_Yaw_Joint", {-3.14, 3.14}},
      {"Left_Wrist_Roll_Joint", {-1.57, 1.57}},
      {"Left_Wrist_Pitch_Joint", {-1.57, 1.57}},
      {"Right_Shoulder_Pitch_Joint", {-3.14, 3.14}},
      {"Right_Shoulder_Roll_Joint", {-3.2, 0.3}},
      {"Right_Shoulder_Yaw_Joint", {-3.14, 3.14}},
      {"Right_Elbow_Pitch_Joint", {-1.8, 0.2}},
      {"Right_Wrist_Yaw_Joint", {-3.14, 3.14}},
      {"Right_Wrist_Roll_Joint", {-1.57, 1.57}},
      {"Right_Wrist_Pitch_Joint", {-1.57, 1.57}},
  };
}

void JointControllerNode::createRosInterfaces() {
  joint_command_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_command",
                                                           rclcpp::QoS(10));
  joint_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
                                                           rclcpp::QoS(10));
  joint_state_subscriber_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", rclcpp::QoS(10),
          std::bind(&JointControllerNode::jointStateCallback, this,
                    std::placeholders::_1));

  if (accept_external_joint_command_) {
    rclcpp::SubscriptionOptions options;
    options.ignore_local_publications = true;
    external_joint_command_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            external_joint_command_topic_, rclcpp::QoS(10),
            std::bind(&JointControllerNode::externalJointCommandCallback, this,
                      std::placeholders::_1),
            options);
    RCLCPP_INFO(this->get_logger(),
                "External joint command subscription enabled on '%s'",
                external_joint_command_topic_.c_str());
  }

  service_ = this->create_service<robot_interface::srv::MotionExecute>(
      "/motion_controller/execute_motion",
      std::bind(&JointControllerNode::handleCloudTask, this,
                std::placeholders::_1, std::placeholders::_2));
}

void JointControllerNode::createTimers() {
  action_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&JointControllerNode::publishJointCommand, this));
  return_to_zero_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&JointControllerNode::publishZeroPositionCommand, this));
  wait_for_zero_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&JointControllerNode::checkZeroAndStartAction, this));
  external_command_watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&JointControllerNode::checkExternalCommandTimeout, this));

  const int hold_period_ms = std::max(
      1, static_cast<int>(1000.0 / std::max(external_hold_publish_hz_, 1.0)));
  external_hold_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(hold_period_ms),
      std::bind(&JointControllerNode::publishExternalHold, this));

  action_timer_->cancel();
  return_to_zero_timer_->cancel();
  wait_for_zero_timer_->cancel();
  external_command_watchdog_timer_->cancel();
  external_hold_timer_->cancel();
}

void JointControllerNode::initializeLastExternalCommand() {
  last_external_command_.name = joint_name_;
  last_external_command_.position.resize(joint_name_.size(), 0.0);
  for (size_t i = 0; i < joint_name_.size(); ++i) {
    auto it = zero_positions_.find(joint_name_[i]);
    if (it != zero_positions_.end()) {
      last_external_command_.position[i] = it->second;
    }
  }
}

void JointControllerNode::enableEthercatIfConfigured() {
  if (use_ethercat_control_) {
    if (!initEthercatController()) {
      RCLCPP_ERROR(this->get_logger(), "EtherCAT controller init failed, fallback to topic-only mode.");
      use_ethercat_control_ = false;
    }
  }
}

void JointControllerNode::setResponse(
    const std::shared_ptr<robot_interface::srv::MotionExecute::Response>& response,
    bool success, const std::string& message) const {
  response->success = success;
  response->message = message;
}

bool JointControllerNode::canAcceptUserMotion() const {
  return current_state_ == ControllerState::IDLE ||
         current_state_ == ControllerState::FOLLOWING_EXTERNAL ||
         current_state_ == ControllerState::HOLDING_EXTERNAL;
}

void JointControllerNode::cancelPendingActionLocked() {
  wait_for_zero_timer_->cancel();
  pending_action_request_ = nullptr;
  pending_action_response_ = nullptr;
}

void JointControllerNode::resetToIdleLocked() {
  action_timer_->cancel();
  return_to_zero_timer_->cancel();
  wait_for_zero_timer_->cancel();
  stopExternalHoldLocked();
  pending_action_request_ = nullptr;
  pending_action_response_ = nullptr;
  current_state_ = ControllerState::IDLE;
  is_returning_to_zero_ = false;
}

sensor_msgs::msg::JointState JointControllerNode::makeCommandMessage(
    const std::vector<std::string>& names,
    const std::vector<double>& positions) const {
  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = this->now();
  message.name = names;
  message.position = positions;
  return message;
}

void JointControllerNode::publishAndSendCommand(
    const sensor_msgs::msg::JointState& msg) {
  sendJointCommandToEthercat(msg);
  joint_command_publisher_->publish(msg);
}

// Service entry point: translate motion_id into one controller operation.
void JointControllerNode::handleCloudTask(
    const std::shared_ptr<robot_interface::srv::MotionExecute::Request> request,
    std::shared_ptr<robot_interface::srv::MotionExecute::Response> response) {
  std::lock_guard<std::mutex> lock(mtx_);
  
  std::string motion_id = request->motion_id;
  auto it = motion_commands_.find(motion_id);
  
  if (it == motion_commands_.end()) {
    setResponse(response, false, "Unknown motion command: " + motion_id);
    RCLCPP_WARN(this->get_logger(), "Unknown motion command: %s", motion_id.c_str());
    return;
  }
  
  MotionCommand command = it->second;
  
  switch (command) {
    case MotionCommand::PLAY_ACTION:
      handlePlayAction(request, response);
      break;
    case MotionCommand::RETURN_TO_ZERO:
      handleReturnToZero(request, response);
      break;
    case MotionCommand::STOP_ACTION:
      handleStopAction(request, response);
      break;
    default:
      setResponse(response, false, "Unhandled motion command: " + motion_id);
      RCLCPP_ERROR(this->get_logger(), "Unhandled motion command: %s", motion_id.c_str());
      break;
  }
}

// Start a stored action. If feedback says the arm is away from zero, return
// to zero first and continue the action from checkZeroAndStartAction().
void JointControllerNode::handlePlayAction(
    const std::shared_ptr<robot_interface::srv::MotionExecute::Request> request,
    std::shared_ptr<robot_interface::srv::MotionExecute::Response> response) {
  if (!canAcceptUserMotion()) {
    setResponse(response, false,
                "Controller is busy, current state: " +
                    stateToString(current_state_));
    RCLCPP_WARN(this->get_logger(), "Rejecting play action request, controller is busy");
    return;
  }
  stopExternalHoldLocked();
  
  if (!all_at_zero_ && current_joint_state_) {
    RCLCPP_INFO(this->get_logger(), 
                "Joints not at zero position, starting return to zero before playing action");
    
    pending_action_request_ = request;
    pending_action_response_ = response;
    
    startReturnToZero();
    
    current_state_ = ControllerState::WAITING_FOR_ZERO;
    wait_for_zero_timer_->reset();
    
    // 注意：这里不设置response，因为会在回零完成后设置
    return;
  }
  
  executeActionDirectly(request, response);
}

// Load action data and arm the 100 Hz publisher.
void JointControllerNode::executeActionDirectly(
    const std::shared_ptr<robot_interface::srv::MotionExecute::Request> request,
    std::shared_ptr<robot_interface::srv::MotionExecute::Response> response) {
  // readData(path_ + request->motion_id);
  if (!readData(path_ + "guojia.data")) {
    setResponse(response, false, "Failed to read action data");
    return;
  }
  
  current_state_ = ControllerState::EXECUTING_ACTION;
  action_timer_->reset();
  return_to_zero_timer_->cancel();
  wait_for_zero_timer_->cancel();
  external_command_watchdog_timer_->cancel();
  
  setResponse(response, true, "Action execution started: " + request->motion_id);
  RCLCPP_INFO(this->get_logger(), "Starting action execution for %s",
              request->motion_id.c_str());
}

// Continuation for actions that first needed a return-to-zero sequence.
void JointControllerNode::checkZeroAndStartAction() {
  std::lock_guard<std::mutex> lock(mtx_);
  
  if (current_state_ != ControllerState::WAITING_FOR_ZERO) {
    wait_for_zero_timer_->cancel();
    return;
  }
  
  if (all_at_zero_) {
    RCLCPP_INFO(this->get_logger(), 
                "Return to zero completed, now starting the requested action");
    
    wait_for_zero_timer_->cancel();
    
    if (pending_action_request_ && pending_action_response_) {
      executeActionDirectly(pending_action_request_, pending_action_response_);
    }
    
    pending_action_request_ = nullptr;
    pending_action_response_ = nullptr;
  }
}

// Start a smooth return-to-zero sequence.
void JointControllerNode::handleReturnToZero(
    const std::shared_ptr<robot_interface::srv::MotionExecute::Request> request,
    std::shared_ptr<robot_interface::srv::MotionExecute::Response> response) {
  (void)request;
  
  if (!canAcceptUserMotion() && current_state_ != ControllerState::WAITING_FOR_ZERO) {
    setResponse(response, false,
                "Controller is busy, current state: " +
                    stateToString(current_state_));
    RCLCPP_WARN(this->get_logger(), "Rejecting return to zero request, controller is busy");
    return;
  }
  
  if (current_state_ == ControllerState::WAITING_FOR_ZERO) {
    cancelPendingActionLocked();
  }
  
  startReturnToZero();
  
  setResponse(response, true, "Return to zero started");
  RCLCPP_INFO(this->get_logger(), "Return to zero started by service request");
}

// Stop whichever operation currently owns the motors and return to IDLE.
void JointControllerNode::handleStopAction(
    const std::shared_ptr<robot_interface::srv::MotionExecute::Request> request,
    std::shared_ptr<robot_interface::srv::MotionExecute::Response> response) {
  (void)request;
  
  // 停止当前动作
  if (current_state_ == ControllerState::EXECUTING_ACTION) {
    action_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Action execution stopped by request");
  }

  if (current_state_ == ControllerState::FOLLOWING_EXTERNAL ||
      current_state_ == ControllerState::HOLDING_EXTERNAL) {
    stopExternalHoldLocked();
    RCLCPP_INFO(this->get_logger(), "External joint command hold/follow stopped by request");
  }
  
  // 停止回零
  if (current_state_ == ControllerState::RETURNING_TO_ZERO) {
    return_to_zero_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Return to zero stopped by request");
  }
  
  // 停止等待回零
  if (current_state_ == ControllerState::WAITING_FOR_ZERO) {
    cancelPendingActionLocked();
    RCLCPP_INFO(this->get_logger(), "Waiting for zero stopped by request");
  }
  
  resetToIdleLocked();
  
  setResponse(response, true, "Action stopped successfully");
  RCLCPP_INFO(this->get_logger(), "Current action/return to zero stopped");
}

bool JointControllerNode::readData(std::string fileName) {
  std::ifstream file(fileName);
  std::string line;
  actionData_.clear();

  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", fileName.c_str());
    return false;
  }

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    std::vector<double> temp;

    while (std::getline(ss, token, ',')) {
      temp.push_back(std::stof(token));
    }

    if (temp.size() >= 14) {
      std::vector<double> last14(temp.begin(), temp.end());
      actionData_.push_back(last14);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid data line in file: %s", fileName.c_str());
    }
  }
  
  if (actionData_.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid data found in file: %s", fileName.c_str());
    return false;
  }
  
  dataIndex_ = 0;
  dataSize_ = actionData_.size();
  file.close();
  
  RCLCPP_INFO(get_logger(), "Loaded %zu action frames", dataSize_);
  return true;
}

void JointControllerNode::publishJointCommand() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (current_state_ != ControllerState::EXECUTING_ACTION) {
    return;
  }
  
  if (dataIndex_ >= dataSize_) {
    RCLCPP_INFO(get_logger(), "Action execution completed");
    current_state_ = ControllerState::IDLE;
    action_timer_->cancel();
    return;
  }
  
  const auto message = makeCommandMessage(joint_name_, actionData_[dataIndex_++]);
  publishAndSendCommand(message);
}

void JointControllerNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  current_joint_state_ = msg;
  
  all_at_zero_ = checkAllJointsAtZero(msg);
  
  std::lock_guard<std::mutex> lock(mtx_);

  // 如果在回零过程中且所有关节都已回零，则停止回零
  if (is_returning_to_zero_ && all_at_zero_) {
    RCLCPP_INFO(this->get_logger(), 
                "Return to zero completed, all joints at zero position");
    stopReturnToZero();
  }
}

sensor_msgs::msg::JointState JointControllerNode::normalizeExternalCommand(
    const sensor_msgs::msg::JointState& msg) {
  sensor_msgs::msg::JointState command = last_external_command_;
  command.header = msg.header;
  if (command.header.stamp.sec == 0 && command.header.stamp.nanosec == 0) {
    command.header.stamp = this->now();
  }

  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
    const std::string& name = msg.name[i];
    for (size_t j = 0; j < command.name.size(); ++j) {
      if (command.name[j] == name) {
        command.position[j] = msg.position[i];
        break;
      }
    }
  }
  // Joints not present in msg keep last commanded position (status hold).

  return command;
}

void JointControllerNode::preemptBlockingMotionForExternalLocked() {
  if (current_state_ == ControllerState::EXECUTING_ACTION) {
    action_timer_->cancel();
    RCLCPP_WARN(this->get_logger(),
                "External /joint_command preempting EXECUTING_ACTION");
  }
  if (current_state_ == ControllerState::RETURNING_TO_ZERO ||
      is_returning_to_zero_) {
    stopReturnToZero();
    RCLCPP_WARN(this->get_logger(),
                "External /joint_command preempting RETURNING_TO_ZERO");
  }
  if (current_state_ == ControllerState::WAITING_FOR_ZERO) {
    wait_for_zero_timer_->cancel();
    pending_action_request_ = nullptr;
    pending_action_response_ = nullptr;
    RCLCPP_WARN(this->get_logger(),
                "External /joint_command preempting WAITING_FOR_ZERO");
  }
}

void JointControllerNode::externalJointCommandCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!accept_external_joint_command_ || !msg) {
    return;
  }
  if (msg->name.empty() || msg->position.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "External joint command ignored: empty name/position");
    return;
  }
  std::string limit_error;
  if (!validateJointCommand(*msg, &limit_error)) {
    RCLCPP_ERROR(this->get_logger(),
                 "External joint command rejected: %s",
                 limit_error.c_str());
    return;
  }

  bool resume_external = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (current_state_ == ControllerState::EXECUTING_ACTION ||
        current_state_ == ControllerState::RETURNING_TO_ZERO ||
        current_state_ == ControllerState::WAITING_FOR_ZERO ||
        is_returning_to_zero_) {
      preemptBlockingMotionForExternalLocked();
    }

    if (current_state_ == ControllerState::IDLE) {
      RCLCPP_INFO(this->get_logger(),
                  "Following external joint commands from '%s'",
                  external_joint_command_topic_.c_str());
      resume_external = true;
    } else if (current_state_ == ControllerState::HOLDING_EXTERNAL) {
      RCLCPP_INFO(this->get_logger(),
                  "Resuming external commands from hold on '%s'",
                  external_joint_command_topic_.c_str());
      external_hold_timer_->cancel();
      resume_external = true;
    }
    current_state_ = ControllerState::FOLLOWING_EXTERNAL;
  }

  sensor_msgs::msg::JointState command = normalizeExternalCommand(*msg);
  if (!validateJointCommand(command, &limit_error)) {
    RCLCPP_ERROR(this->get_logger(),
                 "External joint command rejected (joint limit): %s",
                 limit_error.c_str());
    return;
  }
  last_external_command_ = command;
  last_external_command_time_ = this->now();
  // Only re-enable once when a new external stream starts — NOT every frame (causes violent jitter).
  if (resume_external && use_ethercat_control_) {
    ensureEthercatMotorsEnabled();
  }
  sendJointCommandToEthercat(command);

  {
    std::lock_guard<std::mutex> lock(mtx_);
    external_command_watchdog_timer_->reset();
  }
}

void JointControllerNode::stopExternalHoldLocked() {
  external_hold_timer_->cancel();
  external_command_watchdog_timer_->cancel();
}

void JointControllerNode::publishExternalHold() {
  sensor_msgs::msg::JointState hold_cmd;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (current_state_ != ControllerState::HOLDING_EXTERNAL) {
      external_hold_timer_->cancel();
      return;
    }
    hold_cmd = last_external_command_;
  }
  if (use_ethercat_control_) {
    sendJointCommandToEthercat(hold_cmd);
  }
}

void JointControllerNode::checkExternalCommandTimeout() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (current_state_ != ControllerState::FOLLOWING_EXTERNAL) {
    external_command_watchdog_timer_->cancel();
    return;
  }

  const double idle_s =
      (this->now() - last_external_command_time_).seconds();
  if (idle_s < external_command_idle_timeout_s_) {
    return;
  }

  external_command_watchdog_timer_->cancel();

  if (hold_last_command_on_idle_ && use_ethercat_control_) {
    current_state_ = ControllerState::HOLDING_EXTERNAL;
    external_hold_timer_->reset();
    RCLCPP_INFO(this->get_logger(),
                "No external commands for %.2fs — holding last joint targets (%.0f Hz)",
                idle_s, external_hold_publish_hz_);
    return;
  }

  current_state_ = ControllerState::IDLE;
  RCLCPP_INFO(this->get_logger(),
              "External joint command stream idle for %.2fs, back to IDLE",
              idle_s);
}

bool JointControllerNode::checkAllJointsAtZero(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& joint_name = msg->name[i];
    double current_position = msg->position[i];

    if (zero_positions_.find(joint_name) != zero_positions_.end()) {
      double zero_position = zero_positions_[joint_name];
      if (std::abs(current_position - zero_position) > zero_position_tolerance_) {
        return false;
      }
    }
  }
  return true;
}

void JointControllerNode::stopReturnToZero() {
  if (!is_returning_to_zero_) {
    return;
  }
  
  is_returning_to_zero_ = false;
  
  // 如果当前状态是回零中，且没有等待执行的动作，则回到空闲状态
  if (current_state_ == ControllerState::RETURNING_TO_ZERO && 
      !pending_action_request_) {
    current_state_ = ControllerState::IDLE;
  }
  
  return_to_zero_timer_->cancel();
  
  RCLCPP_INFO(this->get_logger(), "Stopped return to zero sequence");
}

void JointControllerNode::publishZeroPositionCommand() {
  std::lock_guard<std::mutex> lock(mtx_);
  
  if (!is_returning_to_zero_) {
    return;
  }
  
  // 获取当前关节状态
  if (!current_joint_state_) {
    RCLCPP_WARN(this->get_logger(), "No joint state available for return to zero");
    return;
  }
  
  const double elapsed_time =
      (this->now() - return_to_zero_start_time_).seconds();
  
  // 计算插值因子 (0.0 -> 1.0)
  double alpha = std::min(elapsed_time / return_to_zero_duration_, 1.0);
  
  // 应用缓动函数使运动更平滑
  alpha = smoothStep(alpha);

  std::vector<double> target_positions;
  target_positions.reserve(joint_name_.size());
  for (const auto& joint_name : joint_name_) {
    const double zero_position = zero_positions_.at(joint_name);
    // 查找当前关节位置
    double current_position = zero_position;
    
    for (size_t i = 0; i < current_joint_state_->name.size(); ++i) {
      if (current_joint_state_->name[i] == joint_name) {
        current_position = current_joint_state_->position[i];
        break;
      }
    }
    
    // 线性插值从当前位置到零位
    target_positions.push_back(
        current_position + (zero_position - current_position) * alpha);
  }

  const auto command_msg = makeCommandMessage(joint_name_, target_positions);
  publishAndSendCommand(command_msg);
  
  // 如果已经到达目标时间，检查是否完成回零
  if (alpha >= 1.0) {
    if (all_at_zero_) {
      RCLCPP_INFO(this->get_logger(), "Return to zero completed smoothly");
    } else {
      // 如果时间到了但还没到零位，继续发布零位命令直到到达
      RCLCPP_DEBUG(this->get_logger(), "Smooth return completed, fine-tuning to exact zero");
      publishExactZeroPositionCommand();
    }
    stopReturnToZero();
  }
}

void JointControllerNode::publishExactZeroPositionCommand() {
  std::vector<double> zero_positions;
  zero_positions.reserve(joint_name_.size());
  for (const auto& joint_name : joint_name_) {
    zero_positions.push_back(zero_positions_.at(joint_name));
  }

  const auto command_msg = makeCommandMessage(joint_name_, zero_positions);
  publishAndSendCommand(command_msg);
}

double JointControllerNode::smoothStep(double t) {
  return t * t * (3.0 - 2.0 * t);
}

void JointControllerNode::startReturnToZero() {
  if (is_returning_to_zero_) {
    return;
  }
  
  // 记录开始时间
  return_to_zero_start_time_ = this->now();
  
  is_returning_to_zero_ = true;
  current_state_ = ControllerState::RETURNING_TO_ZERO;
  return_to_zero_timer_->reset();
  action_timer_->cancel();
  wait_for_zero_timer_->cancel();
  stopExternalHoldLocked();
  
  RCLCPP_INFO(this->get_logger(), "Starting smooth return to zero sequence (duration: %.1fs)",
              return_to_zero_duration_);
}

std::string JointControllerNode::stateToString(ControllerState state) {
  switch (state) {
    case ControllerState::IDLE: return "IDLE";
    case ControllerState::EXECUTING_ACTION: return "EXECUTING_ACTION";
    case ControllerState::RETURNING_TO_ZERO: return "RETURNING_TO_ZERO";
    case ControllerState::WAITING_FOR_ZERO: return "WAITING_FOR_ZERO";
    case ControllerState::FOLLOWING_EXTERNAL: return "FOLLOWING_EXTERNAL";
    case ControllerState::HOLDING_EXTERNAL: return "HOLDING_EXTERNAL";
    default: return "UNKNOWN";
  }
}

std::vector<std::vector<std::string>> JointControllerNode::armJointGroups() const {
  if (joint_name_.size() <= 7) {
    return {joint_name_};
  }
  return {
      std::vector<std::string>(joint_name_.begin(), joint_name_.begin() + 7),
      std::vector<std::string>(joint_name_.begin() + 7, joint_name_.end()),
  };
}

bool JointControllerNode::configureMotorGroupMitMode(
    const std::vector<std::string>& arm_joints, const char* arm_label,
    int retry_count, int retry_delay_ms) {
#ifndef LENS_ENABLE_ACTUATOR_SDK
  (void)arm_joints;
  (void)arm_label;
  (void)retry_count;
  (void)retry_delay_ms;
  return false;
#else
  if (!actuator_controller_) {
    return false;
  }

  const int attempts = std::max(1, retry_count);
  const int delay_ms = std::max(50, retry_delay_ms);
  std::vector<MotorModeEnum> modes(arm_joints.size(), MotorModeEnum::MODE_MIT);

  for (int attempt = 1; attempt <= attempts; ++attempt) {
    RCLCPP_INFO(this->get_logger(), "Motor init %s arm attempt %d/%d...",
                arm_label, attempt, attempts);

    // disable 失败不阻断；部分轴可能已经在 fault/disabled 状态。
    (void)actuator_controller_->disableMotor(arm_joints);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

    if (actuator_controller_->setMotorMode(arm_joints, modes) != 0) {
      RCLCPP_WARN(this->get_logger(), "%s arm setMotorMode failed on attempt %d",
                  arm_label, attempt);
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    if (actuator_controller_->enableMotor(arm_joints) == 0) {
      RCLCPP_INFO(this->get_logger(), "%s arm motors enabled.", arm_label);
      return true;
    }

    RCLCPP_WARN(this->get_logger(), "%s arm enableMotor failed on attempt %d",
                arm_label, attempt);
  }

  RCLCPP_ERROR(this->get_logger(),
               "%s arm motor init failed after %d attempts. "
               "Check arm power/CAN wiring; Left_Elbow CAN ID 4 error 0x08 "
               "often needs power cycle.",
               arm_label, attempts);
  return false;
#endif
}

bool JointControllerNode::ensureEthercatMotorsEnabled() {
#ifndef LENS_ENABLE_ACTUATOR_SDK
  return false;
#else
  if (!actuator_controller_) {
    return false;
  }
  const char* arm_labels[] = {"left", "right"};
  bool ok = true;
  const auto arm_groups = armJointGroups();
  for (size_t i = 0; i < arm_groups.size(); ++i) {
    const char* label = i < 2 ? arm_labels[i] : "arm";
    if (!configureMotorGroupMitMode(arm_groups[i], label, 1, 80)) {
      ok = false;
    }
  }
  if (ok) {
    consecutive_mit_failures_ = 0;
  }
  return ok;
#endif
}

bool JointControllerNode::initEthercatController() {
#ifndef LENS_ENABLE_ACTUATOR_SDK
  RCLCPP_ERROR(this->get_logger(), "LENS_ENABLE_ACTUATOR_SDK is OFF at build time.");
  return false;
#else
  if (actuator_config_file_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "actuator_config_file is empty.");
    return false;
  }
  if (actuator_params_file_.empty()) {
    RCLCPP_WARN(this->get_logger(), "actuator_params_file empty, SDK will use default params.");
  } else {
    if (!ActuatorController::loadActuatorParamsConfig(actuator_params_file_)) {
      RCLCPP_ERROR(this->get_logger(), "loadActuatorParamsConfig failed: %s", actuator_params_file_.c_str());
      return false;
    }
  }

  actuator_controller_ = std::make_shared<ActuatorController>();
  if (!actuator_controller_->initCanIdAndEthercatRelation(
          ethercat_ifname_, actuator_config_file_, ethercat_send_frequency_)) {
    RCLCPP_ERROR(this->get_logger(), "initCanIdAndEthercatRelation failed.");
    return false;
  }

  const int post_delay_ms = std::max(0, motor_init_post_ec_delay_ms_);
  const int retry_delay_ms = std::max(50, motor_init_retry_delay_ms_);
  if (post_delay_ms > 0) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting %d ms for EtherCAT/CAN bus to stabilize before motor init...",
                post_delay_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(post_delay_ms));
  }

  // boardcast=true 要求按 CAN 端口整臂批量下发（不可单关节），左/右各 7 轴。
  const char* arm_labels[] = {"left", "right"};
  const auto arm_groups = armJointGroups();
  for (size_t i = 0; i < arm_groups.size(); ++i) {
    const char* label = i < 2 ? arm_labels[i] : "arm";
    if (!configureMotorGroupMitMode(
            arm_groups[i], label, motor_init_retry_count_, retry_delay_ms)) {
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "EtherCAT control enabled: if=%s, cfg=%s, freq=%dHz, MIT(kp=%.3f,kd=%.3f,vel=%.3f,tor=%.3f)",
              ethercat_ifname_.c_str(), actuator_config_file_.c_str(), ethercat_send_frequency_,
              mit_kp_, mit_kd_, mit_vel_, mit_torque_);

  const int feedback_ms = std::max(
      1, static_cast<int>(1000.0 / std::max(joint_feedback_hz_, 1.0)));
  joint_feedback_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(feedback_ms),
      std::bind(&JointControllerNode::publishJointFeedback, this));
  RCLCPP_INFO(this->get_logger(), "Publishing /joint_states feedback at %.1f Hz",
              joint_feedback_hz_);
  return true;
#endif
}

void JointControllerNode::publishJointFeedback() {
#ifndef LENS_ENABLE_ACTUATOR_SDK
  return;
#else
  if (!use_ethercat_control_ || !actuator_controller_) {
    return;
  }

  std::vector<double> positions(joint_name_.size(), 0.0);
  if (actuator_controller_->getCurrentPosition(joint_name_, positions) != 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "getCurrentPosition failed");
    return;
  }

  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = this->now();
  message.name = joint_name_;
  message.position.resize(positions.size());
  for (size_t i = 0; i < positions.size() && i < joint_name_.size(); ++i) {
    message.position[i] =
        motorToRosPosition(joint_name_[i], positions[i]);
  }
  joint_state_publisher_->publish(message);
#endif
}

bool JointControllerNode::validateJointCommand(
    const sensor_msgs::msg::JointState& msg, std::string* error) const {
  if (!enforce_joint_position_limits_) {
    return true;
  }
  if (msg.name.size() != msg.position.size()) {
    if (error) {
      *error = "name/position size mismatch";
    }
    return false;
  }
  for (size_t i = 0; i < msg.name.size(); ++i) {
    const std::string& joint_name = msg.name[i];
    const double q = msg.position[i];
    if (!std::isfinite(q)) {
      if (error) {
        *error = joint_name + " non-finite position";
      }
      return false;
    }
    const auto it = joint_position_limits_.find(joint_name);
    if (it == joint_position_limits_.end()) {
      if (error) {
        *error = "unknown joint " + joint_name;
      }
      return false;
    }
    const double lo = it->second.first;
    const double hi = it->second.second;
    if (q < lo - 1e-9 || q > hi + 1e-9) {
      if (error) {
        *error = joint_name + " out of range: " + std::to_string(q) +
                 " rad, allowed [" + std::to_string(lo) + ", " +
                 std::to_string(hi) + "]";
      }
      return false;
    }
  }
  return true;
}

void JointControllerNode::loadInvertJointNamesParam() {
  invert_joint_names_.clear();
  const std::string raw =
      this->get_parameter("invert_joint_names").as_string();
  std::stringstream ss(raw);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = trimCopy(token);
    if (!token.empty()) {
      invert_joint_names_.insert(token);
    }
  }
  if (!invert_joint_names_.empty()) {
    std::string listed;
    for (const auto& name : invert_joint_names_) {
      if (!listed.empty()) {
        listed += ", ";
      }
      listed += name;
    }
    RCLCPP_WARN(this->get_logger(),
                "invert_joint_names active (%zu): %s — ROS/MuJoCo q is negated "
                "when sending to motor and when publishing /joint_states",
                invert_joint_names_.size(), listed.c_str());
  }
}

bool JointControllerNode::isInvertedJoint(const std::string& joint_name) const {
  return invert_joint_names_.find(joint_name) != invert_joint_names_.end();
}

double JointControllerNode::rosToMotorPosition(const std::string& joint_name,
                                               double q_ros) const {
  return isInvertedJoint(joint_name) ? -q_ros : q_ros;
}

double JointControllerNode::motorToRosPosition(const std::string& joint_name,
                                               double q_motor) const {
  return isInvertedJoint(joint_name) ? -q_motor : q_motor;
}

void JointControllerNode::sendJointCommandToEthercat(const sensor_msgs::msg::JointState& msg) {
  if (!use_ethercat_control_) {
    return;
  }
#ifndef LENS_ENABLE_ACTUATOR_SDK
  (void)msg;
  return;
#else
  if (!actuator_controller_) {
    return;
  }

  std::vector<std::string> names;
  std::vector<double> pos, vel, tor, kp, kd;
  names.reserve(msg.name.size());
  pos.reserve(msg.position.size());
  vel.reserve(msg.position.size());
  tor.reserve(msg.position.size());
  kp.reserve(msg.position.size());
  kd.reserve(msg.position.size());
  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
    names.push_back(msg.name[i]);
    pos.push_back(rosToMotorPosition(msg.name[i], msg.position[i]));
    vel.push_back(mit_vel_);
    tor.push_back(mit_torque_);
    kp.push_back(mit_kp_);
    kd.push_back(mit_kd_);
  }
  if (names.empty()) {
    return;
  }
  int ret = actuator_controller_->setTargetMit(names, pos, vel, tor, kp, kd);
  if (ret != 0) {
    consecutive_mit_failures_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "setTargetMit failed, ret=%d (failures=%d)",
                         ret, consecutive_mit_failures_);
    if (consecutive_mit_failures_ >= 8) {
      consecutive_mit_failures_ = 0;
      RCLCPP_WARN(this->get_logger(),
                  "Repeated MIT failures — attempting motor re-enable");
      ensureEthercatMotorsEnabled();
    }
  } else if (consecutive_mit_failures_ > 0) {
    consecutive_mit_failures_ = 0;
  }
#endif
}
