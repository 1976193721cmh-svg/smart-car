#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "pin_def.h"

/*============================================================================*/
/*                              电机PWM参数配置                               */
/*============================================================================*/

/*
 * PWM频率 = 20kHz（20000Hz）
 * 计算过程：72MHz / (36预分频) / (100自动重装) = 20000Hz
 * 人耳听不到20kHz的噪音，所以电机不会发出"滋滋"声
 */
#define MOTOR_PWM_FREQ        20000    // PWM频率：20kHz
#define MOTOR_PWM_PERIOD      100      // PWM周期（ARR值）：0~100，对应0%~100%占空比

#define MOTOR_SPEED_MIN       0        // 电机最小速度：0（停止）
#define MOTOR_SPEED_MAX       100      // 电机最大速度：100（全速）

/*
 * 转弯速度比例系数
 * 转弯时，内侧轮速度 = 外侧轮速度 × TURN_SPEED_RATIO
 * 比如外侧轮速度80，内侧轮速度 = 80 × 0.1 = 8
 * 这样转弯更平滑，不会原地打转
 */
#define TURN_SPEED_RATIO      0.1f

/*============================================================================*/
/*                              数据结构定义                                   */
/*============================================================================*/

/*
 * Motor_State - 单个电机的状态
 * 记录每个电机的速度、方向和运行状态
 */
typedef struct {
    uint8_t speed;              // 当前速度（0~100）
    Motor_Direction dir;        // 当前方向（正转/反转/停止）
    uint8_t is_running;         // 是否在运行（1=运行中，0=已停止）
} Motor_State;

/*
 * Car_State - 整车状态
 * 包含4个电机的状态，以及整车的速度和方向
 */
typedef struct {
    Motor_State motor_lf;       // 左前轮状态
    Motor_State motor_lb;       // 左后轮状态
    Motor_State motor_rf;       // 右前轮状态
    Motor_State motor_rb;       // 右后轮状态
    uint8_t car_speed;          // 整车速度（0~100）
    uint8_t car_direction;      // 整车方向（1=前进，2=后退，3=左转，4=右转，5~8=带弧度的转弯）
} Car_State;

/*============================================================================*/
/*                              函数声明                                       */
/*============================================================================*/

/* ---- 基本电机控制 ---- */
void Motor_Init(void);                                          // 初始化所有电机（GPIO+PWM）
void Motor_SetSpeed(Motor_ID motor, uint8_t speed);             // 设置单个电机速度（0~100）
void Motor_SetDirection(Motor_ID motor, Motor_Direction dir);   // 设置单个电机方向
void Motor_Stop(Motor_ID motor);                                // 停止单个电机
void Motor_StopAll(void);                                       // 停止所有电机

/* ---- 轮组控制（左轮组/右轮组） ---- */
void Motor_SetWheelSpeed(Wheel_Group wheel, uint8_t speed);             // 设置一侧轮子的速度
void Motor_SetWheelDirection(Wheel_Group wheel, Motor_Direction dir);   // 设置一侧轮子的方向
void Motor_StopWheel(Wheel_Group wheel);                                 // 停止一侧轮子

/* ---- 高级运动控制（整车控制） ---- */
void Car_Forward(uint8_t speed);                // 直行前进
void Car_Backward(uint8_t speed);               // 直行后退
void Car_TurnLeft(uint8_t speed);               // 原地左转（左轮后退，右轮前进）
void Car_TurnRight(uint8_t speed);              // 原地右转（右轮后退，左轮前进）
void Car_ForwardTurnLeft(uint8_t speed);        // 前进中左转（左轮减速，右轮全速）
void Car_ForwardTurnRight(uint8_t speed);       // 前进中右转（右轮减速，左轮全速）
void Car_BackwardTurnLeft(uint8_t speed);       // 后退中左转
void Car_BackwardTurnRight(uint8_t speed);      // 后退中右转
void Car_Stop(void);                            // 停车
void Car_SetSpeed(uint8_t speed);               // 保持当前方向，只改变速度
void Car_ChangeSpeed(int8_t delta);             // 在当前速度基础上增加/减少（delta可为正负数）

/* ---- 状态获取 ---- */
uint8_t Motor_GetSpeed(Motor_ID motor);                 // 获取单个电机速度
Motor_Direction Motor_GetDirection(Motor_ID motor);     // 获取单个电机方向
uint8_t Motor_GetWheelSpeed(Wheel_Group wheel);         // 获取一侧轮子的平均速度
Motor_Direction Motor_GetWheelDirection(Wheel_Group wheel); // 获取一侧轮子的方向
Car_State* Car_GetState(void);                          // 获取整车状态指针

#endif
