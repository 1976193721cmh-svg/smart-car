/**
  ******************************************************************************
  * @file    pin_def.h
  * @author  SmartCar Team
  * @version V2.1
  * @date    2026-04-27
  * @brief   智能小车引脚定义文件
  *          基于STM32F103C8T6核心板，2个TB6612驱动4个直流电机方案
  *          TB6612 #1: 控制左轮（左前+左后），使用TIM2
  *          TB6612 #2: 控制右轮（右前+右后），使用TIM3
  ******************************************************************************
  */

#ifndef __PIN_DEF_H
#define __PIN_DEF_H

#include "stm32f10x.h"

/*============================================================================*/
/*                              TB6612 #1 左轮电机驱动引脚                     */
/*                          （左前轮 + 左后轮，使用TIM2）                      */
/*============================================================================*/

// 左前轮控制
#define MOTOR_LF_PWM_PIN          GPIO_Pin_0      // PA0: TIM2_CH1（PWM输出，控制左前轮速度）
#define MOTOR_LF_PWM_PORT         GPIOA
#define MOTOR_LF_PWM_TIM          TIM2
#define MOTOR_LF_PWM_CHANNEL      TIM_Channel_1

#define MOTOR_LF_IN1_PIN          GPIO_Pin_1      // PA1: 左前轮方向控制引脚1
#define MOTOR_LF_IN1_PORT         GPIOA

#define MOTOR_LF_IN2_PIN          GPIO_Pin_2      // PA2: 左前轮方向控制引脚2
#define MOTOR_LF_IN2_PORT         GPIOA

// 左后轮控制
#define MOTOR_LB_PWM_PIN          GPIO_Pin_3      // PA3: TIM2_CH4（PWM输出，控制左后轮速度）
#define MOTOR_LB_PWM_PORT         GPIOA
#define MOTOR_LB_PWM_TIM          TIM2
#define MOTOR_LB_PWM_CHANNEL      TIM_Channel_4

#define MOTOR_LB_IN1_PIN          GPIO_Pin_4      // PA4: 左后轮方向控制引脚1
#define MOTOR_LB_IN1_PORT         GPIOA

#define MOTOR_LB_IN2_PIN          GPIO_Pin_5      // PA5: 左后轮方向控制引脚2
#define MOTOR_LB_IN2_PORT         GPIOA

/*============================================================================*/
/*                              TB6612 #2 右轮电机驱动引脚                     */
/*                          （右前轮 + 右后轮，使用TIM3）                      */
/*============================================================================*/

// 右前轮控制
#define MOTOR_RF_PWM_PIN          GPIO_Pin_0      // PB0: TIM3_CH3（PWM输出，控制右前轮速度）
#define MOTOR_RF_PWM_PORT         GPIOB
#define MOTOR_RF_PWM_TIM          TIM3
#define MOTOR_RF_PWM_CHANNEL      TIM_Channel_3

#define MOTOR_RF_IN1_PIN          GPIO_Pin_1      // PB1: 右前轮方向控制引脚1
#define MOTOR_RF_IN1_PORT         GPIOB

#define MOTOR_RF_IN2_PIN          GPIO_Pin_3      // PB3: 右前轮方向控制引脚2（注意：PB3是JTDO，需禁用JTAG才能用）
#define MOTOR_RF_IN2_PORT         GPIOB

// 右后轮控制
#define MOTOR_RB_PWM_PIN          GPIO_Pin_6      // PA6: TIM3_CH1（PWM输出，控制右后轮速度）
#define MOTOR_RB_PWM_PORT         GPIOA
#define MOTOR_RB_PWM_TIM          TIM3
#define MOTOR_RB_PWM_CHANNEL      TIM_Channel_1

#define MOTOR_RB_IN1_PIN          GPIO_Pin_5      // PB5: 右后轮方向控制引脚1
#define MOTOR_RB_IN1_PORT         GPIOB

#define MOTOR_RB_IN2_PIN          GPIO_Pin_6      // PB6: 右后轮方向控制引脚2
#define MOTOR_RB_IN2_PORT         GPIOB

// TB6612使能引脚（STBY）直接接3.3V，无需控制

/*============================================================================*/
/*                              SG90舵机引脚定义                              */
/*============================================================================*/

// PB7: TIM4_CH2，用于SG90舵机PWM控制
#define SERVO_PWM_PIN             GPIO_Pin_7      // PB7: TIM4_CH2
#define SERVO_PWM_PORT            GPIOB
#define SERVO_PWM_TIM             TIM4
#define SERVO_PWM_CHANNEL         TIM_Channel_2

/*============================================================================*/
/*                              OLED显示屏引脚定义(I2C)                       */
/*============================================================================*/

#define OLED_I2C                  I2C1
#define OLED_I2C_SCL_PIN          GPIO_Pin_8      // PB8: I2C1_SCL（时钟线）
#define OLED_I2C_SCL_PORT         GPIOB
#define OLED_I2C_SDA_PIN          GPIO_Pin_9      // PB9: I2C1_SDA（数据线）
#define OLED_I2C_SDA_PORT         GPIOB

/*============================================================================*/
/*                              TCRT5000红外传感器引脚定义                     */
/*============================================================================*/

#define INFRARED_1_PIN            GPIO_Pin_12     // PB12: 红外传感器1 DO（数字输出）
#define INFRARED_1_PORT           GPIOB

#define INFRARED_2_PIN            GPIO_Pin_13     // PB13: 红外传感器2 DO
#define INFRARED_2_PORT           GPIOB

#define INFRARED_3_PIN            GPIO_Pin_14     // PB14: 红外传感器3 DO
#define INFRARED_3_PORT           GPIOB

#define INFRARED_4_PIN            GPIO_Pin_15     // PB15: 红外传感器4 DO
#define INFRARED_4_PORT           GPIOB

/*============================================================================*/
/*                              HC-SR04超声波模块引脚定义                     */
/*============================================================================*/

#define ULTRASONIC_TRIG_PIN       GPIO_Pin_8      // PA8: 触发信号(输出)，给高电平10us触发测距
#define ULTRASONIC_TRIG_PORT      GPIOA

#define ULTRASONIC_ECHO_PIN       GPIO_Pin_11     // PA11: 回波信号(输入)，接收超声波返回的高电平脉冲
#define ULTRASONIC_ECHO_PORT      GPIOA

/*============================================================================*/
/*                              HC-06蓝牙模块引脚定义(USART1)                 */
/*============================================================================*/

#define BLUETOOTH_USART           USART1
#define BLUETOOTH_TX_PIN          GPIO_Pin_9      // PA9: USART1_TX（蓝牙模块的RXD）
#define BLUETOOTH_TX_PORT         GPIOA
#define BLUETOOTH_RX_PIN          GPIO_Pin_10     // PA10: USART1_RX（蓝牙模块的TXD）
#define BLUETOOTH_RX_PORT         GPIOA

/*============================================================================*/
/*                              系统LED引脚定义                               */
/*============================================================================*/

#define LED_PC13_PIN              GPIO_Pin_13     // PC13: 板载LED（低电平点亮）
#define LED_PC13_PORT             GPIOC

/*============================================================================*/
/*                              引脚功能宏定义                                */
/*============================================================================*/

// 电机方向定义
typedef enum
{
    MOTOR_DIR_FORWARD  = 0,    // 正转（前进）
    MOTOR_DIR_BACKWARD = 1,    // 反转（后退）
    MOTOR_DIR_STOP     = 2     // 停止（刹车）
} Motor_Direction;

// 小车轮组定义（兼容旧代码）
typedef enum
{
    WHEEL_LEFT  = 0,           // 左轮组（左前+左后）
    WHEEL_RIGHT = 1            // 右轮组（右前+右后）
} Wheel_Group;

// 单个电机编号
typedef enum
{
    MOTOR_LF = 0,              // 左前轮（Left Front）
    MOTOR_LB = 1,              // 左后轮（Left Back）
    MOTOR_RF = 2,              // 右前轮（Right Front）
    MOTOR_RB = 3               // 右后轮（Right Back）
} Motor_ID;

// 红外传感器编号
typedef enum
{
    IR_FRONT_LEFT   = 0,       // 前左红外
    IR_FRONT_RIGHT  = 1,       // 前右红外
    IR_LEFT_SIDE    = 2,       // 左侧红外
    IR_RIGHT_SIDE   = 3        // 右侧红外
} Infrared_ID;

#endif /* __PIN_DEF_H */
