/*
 * @Description:
 * @Version: V1.0
 * @Author: zw_1520@163.com
 * @Date: 2025-08-18 09:41:26
 * @LastEditors: zw_1520@163.com
 * @LastEditTime: 2025-08-18 10:45:16
 * Copyright (C) 2024-2050 Lens All rights reserved.
 */

#ifndef __ACTUATOR_CONTROLLER_H__
#define __ACTUATOR_CONTROLLER_H__
#include <vector>
#include <string>
#include "motor_mode_enum.h"

// 前向声明
class ActuatorControllerPrivate;

// 辅助函数：将浮点数转换为位值
typedef unsigned short uint16_t;

class ActuatorController {
public:
    ActuatorController();
    ~ActuatorController();

    /**
     * @brief 初始化CAN ID和EtherCAT从站关系
     * @param ifname 网络接口名称
     * @param config_file JSON配置文件路径
     * @param sendFrequency EtherCAT发送频率，单位Hz，默认500Hz
     * @return true 成功, false 失败
     */
    bool initCanIdAndEthercatRelation(const std::string& ifname, const std::string& config_file, int send_frequency = 500);

    /**
     * @brief 加载执行器参数配置
     * @param params_file 执行器参数配置文件路径
     * @return true 成功, false 失败
     */
    static bool loadActuatorParamsConfig(const std::string& params_file = std::string("./actuator_params_config.json"));

public:
    /**
     * @brief 设置日志输出级别
     * @param level 日志级别 (0-TRACE, 1-DEBUG, 2-INFO, 3-WARNING, 4-ERROR, 5-CRITICAL)
     */
    static void setLogLevel(int level);
    
    /**
     * @brief 设置日志输出到文件
     * @param log_file 日志文件路径，为空则只输出到控制台
     * @param truncate 是否清空已有文件
     */
    static void setLogFile(const std::string& log_file, bool truncate = false);
    
    // Sets target torque for actuator
    int setTargetMit(const std::string& joint_name, double position, double velocity, double torque, double kp, double kd);
    int setTargetMit(const std::vector<std::string>& joint_names, const std::vector<double>& positions, const std::vector<double>& velocities, const std::vector<double>& torques, const std::vector<double>& kps, const std::vector<double>& kds);

    // Moves actuator to zero position
    int moveToZero(const std::string& joint_name);
    int moveToZero(const std::vector<std::string>& joint_names);

    // Sets target position for actuator
    int setTargetPosition(const std::string& joint_name, double target_position);
    int setTargetPosition(const std::vector<std::string>& joint_names, const std::vector<double>& target_positions);

    // Sets target velocity for actuator
    int setTargetVelocity(const std::string& joint_name, double target_velocity);
    int setTargetVelocity(const std::vector<std::string>& joint_names, const std::vector<double>& target_velocities);

    // Sets target torque for actuator
    int setTargetTorque(const std::string& joint_name, double target_torque);
    int setTargetTorque(const std::vector<std::string>& joint_names, const std::vector<double>& target_torques);

    // Sets target current for actuator
    int setTargetCurrent(const std::string& joint_name, double target_current);
    int setTargetCurrent(const std::vector<std::string>& joint_names, const std::vector<double>& target_currents);

    // Sets target position and velocity for actuator
    int setTargetPositionAndVelocity(const std::string& joint_name, double target_position, double target_velocity);
    int setTargetPositionAndVelocity(const std::vector<std::string>& joint_names, const std::vector<double>& target_positions, const std::vector<double>& target_velocities);

    // Gets current position of actuator
    int getCurrentPosition(const std::string& joint_name, double &current_position);
    int getCurrentPosition(const std::vector<std::string>& joint_names, std::vector<double>& current_positions);

    // Gets current velocity of actuator
    int getCurrentVelocity(const std::string& joint_name, double &current_velocity);
    int getCurrentVelocity(const std::vector<std::string>& joint_names, std::vector<double>& current_velocities);

    // Gets current torque of actuator
    int getCurrentTorque(const std::string& joint_name, double &current_torque);
    int getCurrentTorque(const std::vector<std::string>& joint_names, std::vector<double>& current_torques);

    // Gets current current of actuator
    int getCurrentCurrent(const std::string& joint_name, double &current_current);
    int getCurrentCurrent(const std::vector<std::string>& joint_names, std::vector<double>& current_currents);

    // Enables motor for actuator
    int enableMotor(const std::string& joint_name);
    int enableMotor(const std::vector<std::string>& joint_names);

    // Disables motor for actuator
    int disableMotor(const std::string& joint_name);
    int disableMotor(const std::vector<std::string>& joint_names);

    // Sets motor mode for actuator
    int setMotorMode(const std::string& joint_name, MotorModeEnum mode);
    int setMotorMode(const std::vector<std::string>& joint_names, const std::vector<MotorModeEnum>& modes);

    // Gets motor enable state of actuator
    int getMotorEnableState(const std::string& joint_name, bool &enable_state);
    int getMotorEnableState(const std::vector<std::string>& joint_names, std::vector<bool>& enable_states);

    // Gets motor mode of actuator
    int getMotorMode(const std::string& joint_name, MotorModeEnum &mode);
    int getMotorMode(const std::vector<std::string>& joint_names, std::vector<MotorModeEnum>& modes);
    
    // Sets home position for actuator
    int setHome(const std::string& joint_name);
    int setHome(const std::vector<std::string>& joint_names);

private:
    // 私有类指针
    ActuatorControllerPrivate *d_ptr;

    // 禁止拷贝构造和赋值操作
    ActuatorController(const ActuatorController &);
    ActuatorController &operator=(const ActuatorController &);

};

#endif // __ACTUATOR_CONTROLLER_H__