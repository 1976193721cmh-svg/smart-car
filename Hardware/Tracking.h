#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"
#include "pin_def.h"

/*============================================================================*/
/*                              循迹传感器引脚定义                             */
/*============================================================================*/

/*
 * 传感器布局（从左到右）：
 *   PB14(左2)  PB12(左1)  PB13(右1)  PB15(右2)
 *    ←—— 小车前进方向 ——→
 * 
 * 黑线检测：传感器检测到黑线时输出高电平（1）
 * 白底检测：传感器检测到白底时输出低电平（0）
 */

#define TRACK_SENSOR_L2_PIN     GPIO_Pin_14     // PB14: 左2（最左边）
#define TRACK_SENSOR_L2_PORT    GPIOB
#define TRACK_SENSOR_L1_PIN     GPIO_Pin_12     // PB12: 左1
#define TRACK_SENSOR_L1_PORT    GPIOB
#define TRACK_SENSOR_R1_PIN     GPIO_Pin_13     // PB13: 右1
#define TRACK_SENSOR_R1_PORT    GPIOB
#define TRACK_SENSOR_R2_PIN     GPIO_Pin_15     // PB15: 右2（最右边）
#define TRACK_SENSOR_R2_PORT    GPIOB

/*============================================================================*/
/*                              循迹速度参数                                   */
/*============================================================================*/

#define TRACK_SPEED_DEFAULT     70      // 直行速度（较快）
#define TRACK_SPEED_TURN        60      // 原地转向速度（较慢，防止甩出去）
#define TRACK_SPEED_SLOW        60      // 微调速度（前进中微调）

/*============================================================================*/
/*                              传感器状态位掩码                               */
/*============================================================================*/

/*
 * Tracking_Read()返回值格式（bit3~bit0）：
 *   bit3 = 左2（最左边）
 *   bit2 = 左1
 *   bit1 = 右1
 *   bit0 = 右2（最右边）
 * 
 * 例如：
 *   0b0101 → 左1和右2检测到黑线
 *   0b0110 → 左1和右1检测到黑线（在中间，理想状态）
 *   0b1111 → 全黑（到达终点）
 *   0b0000 → 全白（没有线）
 */

#define TRACK_L2_MASK           0x08    // 左2: bit3
#define TRACK_L1_MASK           0x04    // 左1: bit2
#define TRACK_R1_MASK           0x02    // 右1: bit1
#define TRACK_R2_MASK           0x01    // 右2: bit0

/*============================================================================*/
/*                              函数声明                                       */
/*============================================================================*/

void Tracking_Init(void);               // 初始化循迹传感器GPIO
uint8_t Tracking_Read(void);            // 读取4个传感器状态
void Tracking_Run(void);                // 循迹运行（根据传感器状态控制电机）

#endif
