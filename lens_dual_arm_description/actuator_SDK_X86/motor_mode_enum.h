#ifndef __MOTOR_MODE_ENUM_H__
#define __MOTOR_MODE_ENUM_H__

/**
 * @brief 电机控制模式枚举
 * 
 * 定义了支持的各种电机控制模式
 */
typedef enum {
    MODE_TORQUE,          // 扭矩模式
    MODE_MIT,             // MIT模式
    MODE_VELOCITY,        // 速度模式
    MODE_PROFILE_VELOCITY, // 轮廓速度模式
    MODE_POSITION,        // 位置模式
    MODE_PROFILE_POSITION // 轮廓位置模式
} MotorModeEnum;

#endif // __MOTOR_MODE_ENUM_H__