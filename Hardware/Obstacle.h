#ifndef __OBSTACLE_H
#define __OBSTACLE_H

#include "stm32f10x.h"
#include "pin_def.h"

/*============================================================================*/
/*                              超声波测距参数                                 */
/*============================================================================*/

/*
 * HC-SR04超声波模块工作原理：
 * 
 * 1. 给TRIG引脚一个10us以上的高电平脉冲，触发测距
 * 2. 模块自动发送8个40kHz的超声波脉冲
 * 3. 模块检测回波，ECHO引脚输出高电平
 * 4. ECHO高电平持续时间 = 距离 × 2 / 声速
 * 5. 距离(cm) = 高电平时间(us) / 58
 * 
 * 例如：
 *   10cm → 高电平持续 580us
 *   100cm → 高电平持续 5800us
 *   400cm → 高电平持续 23200us
 * 
 * 注意：2020款HC-SR04两次测量间隔不小于200ms
 *       否则测量结果可能不准确
 */

#define ULTRASONIC_TIMEOUT      30000   // 超时计数（对应约50cm）
#define OBSTACLE_THRESHOLD      15      // 障碍物阈值（cm），小于此值认为有障碍
#define OBSTACLE_SAFE_DIST      10      // 安全距离（cm），小于此值需要紧急避让

/* 避障方向 */
#define AVOID_DIR_NONE          0       // 无障碍
#define AVOID_DIR_LEFT          1       // 左边有空隙
#define AVOID_DIR_RIGHT         2       // 右边有空隙
#define AVOID_DIR_BACK          3       // 前后都有障碍

/*============================================================================*/
/*                              避障状态机                                     */
/*============================================================================*/

/*
 * 避障状态机说明：
 * 
 *   IDLE（空闲）→ 检测到障碍 → SCAN（扫描左右）
 *   SCAN → 左边通畅 → TURN_LEFT（左转）
 *   SCAN → 右边通畅 → TURN_RIGHT（右转）
 *   SCAN → 两边都堵 → BACK（后退）
 *   TURN_LEFT/TURN_RIGHT/BACK → FORWARD（前进一段）
 *   FORWARD → IDLE（继续检测）
 */

typedef enum {
    AVOID_STATE_IDLE = 0,       // 空闲状态：正前方检测，无障碍则直行
    AVOID_STATE_SCAN,           // 扫描状态：舵机左右转动，测量左右距离
    AVOID_STATE_TURN_LEFT,      // 左转避障：左边通畅，向左转
    AVOID_STATE_TURN_RIGHT,     // 右转避障：右边通畅，向右转
    AVOID_STATE_BACK,           // 后退状态：两边都有障碍，先后退
    AVOID_STATE_FORWARD         // 前进状态：转完后前进一段，回到IDLE
} Avoid_State;

/*============================================================================*/
/*                              数据结构定义                                   */
/*============================================================================*/

/*
 * Ultrasonic_Data - 超声波测距结果
 * 存储三个方向的测量距离和障碍判断
 */
typedef struct {
    uint16_t distance_front;    // 前方距离(cm)
    uint16_t distance_left;     // 左方距离(cm)
    uint16_t distance_right;    // 右方距离(cm)
    uint8_t  obstacle_front;    // 前方有障碍（1=有，0=无）
    uint8_t  obstacle_left;     // 左方有障碍
    uint8_t  obstacle_right;    // 右方有障碍
} Ultrasonic_Data;

/* 全局变量声明 */
extern Ultrasonic_Data g_ultrasonic;    // 超声波测距结果

/*============================================================================*/
/*                              函数声明                                       */
/*============================================================================*/

void Obstacle_Init(void);               // 初始化超声波模块和舵机
uint16_t Ultrasonic_Measure(void);      // 超声波测距（单次测量，返回cm）
void Obstacle_Run(void);                // 避障运行函数（在main循环中周期调用）

#endif
