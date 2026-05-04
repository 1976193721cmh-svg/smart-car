#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"
#include "pin_def.h"

/*============================================================================*/
/*                              SG90舵机参数说明                               */
/*============================================================================*/

/*
 * SG90舵机控制原理：
 * 
 * 1. 需要50Hz的PWM信号（周期20ms）
 * 2. 通过改变高电平脉宽来控制角度
 * 
 * 角度与脉宽对应关系：
 *   0度   → 高电平 0.5ms  → 占空比 2.5%  → CCR=499
 *   90度  → 高电平 1.5ms  → 占空比 7.5%  → CCR=1499
 *   180度 → 高电平 2.5ms  → 占空比 12.5% → CCR=2499
 * 
 * 线性插值公式：
 *   CCR = 499 + angle × (2499-499)/180
 *       = 499 + angle × 2000/180
 *       = 499 + angle × 11.11
 * 
 * 定时器配置（TIM4_CH2，PB7）：
 *   系统时钟 = 72MHz
 *   预分频 = 72-1 → 72MHz/72 = 1MHz（每1us计数一次）
 *   自动重装 = 20000-1 → 1MHz/20000 = 50Hz（周期20ms）
 *   CCR范围：499~2499，对应0.5ms~2.5ms
 */

#define SERVO_ANGLE_MIN         0       // 最小角度：0度
#define SERVO_ANGLE_MAX         180     // 最大角度：180度
#define SERVO_ANGLE_MID         90      // 中间角度：90度

/* CCR值（比较值，决定高电平脉宽） */
#define SERVO_CCR_0DEG          499     // 0度对应的CCR值（0.5ms）
#define SERVO_CCR_90DEG         1499    // 90度对应的CCR值（1.5ms）
#define SERVO_CCR_180DEG        2499    // 180度对应的CCR值（2.5ms）

/* 避障扫描角度 */
#define SERVO_ANGLE_FRONT       90      // 正前方（舵机指向前方）
#define SERVO_ANGLE_LEFT        180     // 最左边（舵机指向左方）
#define SERVO_ANGLE_RIGHT       0       // 最右边（舵机指向右方）

/*============================================================================*/
/*                              函数声明                                       */
/*============================================================================*/

void Servo_Init(void);                  // 初始化舵机PWM（TIM4_CH2，PB7）
void Servo_SetAngle(uint8_t angle);     // 设置舵机角度（0~180度）
void Servo_SetCCR(uint16_t ccr);        // 直接设置CCR值（499~2499）

#endif
