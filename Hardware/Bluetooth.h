#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f10x.h"
#include "pin_def.h"

/*============================================================================*/
/*                              缓冲区大小定义                                 */
/*============================================================================*/

#define BT_RX_BUFFER_SIZE       64      // 蓝牙接收环形缓冲区大小
#define BT_CMD_BUFFER_SIZE      16      // 速度命令解析缓冲区大小

/*============================================================================*/
/*                              蓝牙连接状态                                   */
/*============================================================================*/

#define BT_STATE_DISCONNECTED   0       // 蓝牙未连接
#define BT_STATE_CONNECTED      1       // 蓝牙已连接

/*============================================================================*/
/*                              蓝牙通信协议                                   */
/*============================================================================*/

/*
 * 通信协议说明（手机APP → 单片机）：
 * 
 * 1. 方向控制命令（单字节）：
 *    0x00 = 停止
 *    0x01 = 前进
 *    0x02 = 前进右转
 *    0x03 = 原地右转
 *    0x04 = 后退右转
 *    0x05 = 后退
 *    0x06 = 后退左转
 *    0x07 = 原地左转
 *    0x08 = 前进左转
 * 
 * 2. 速度控制命令（多字节）：
 *    0x50 [速度值字节...] 0x59
 *    0x50 = 速度命令头
 *    0x59 = 速度命令尾
 *    速度值范围：0~100
 *    最后一个字节作为实际速度值
 * 
 * 3. 模式切换命令（4个相同字节）：
 *    0x11 0x11 0x11 0x11 = 循迹模式
 *    0x22 0x22 0x22 0x22 = 避障模式
 *    0x33 0x33 0x33 0x33 = 手动模式
 *    连续收到4个相同字节才切换，防止误触发
 */

/* ---- 方向命令 ---- */
#define BT_CMD_STOP             0x00    // 停止
#define BT_CMD_FORWARD          0x01    // 前进
#define BT_CMD_FORWARD_RIGHT    0x02    // 前进右转
#define BT_CMD_RIGHT            0x03    // 原地右转
#define BT_CMD_BACKWARD_RIGHT   0x04    // 后退右转
#define BT_CMD_BACKWARD         0x05    // 后退
#define BT_CMD_BACKWARD_LEFT    0x06    // 后退左转
#define BT_CMD_LEFT             0x07    // 原地左转
#define BT_CMD_FORWARD_LEFT     0x08    // 前进左转

/* ---- 速度命令 ---- */
#define BT_SPEED_HEADER         0x50    // 速度命令头
#define BT_SPEED_TAIL           0x59    // 速度命令尾
#define BT_SPEED_MIN            0       // 最小速度
#define BT_SPEED_MAX            100     // 最大速度

/* ---- 超时时间 ---- */
#define BT_CMD_TIMEOUT_MS       500     // 命令超时时间（ms），超过此时间未收到命令则停车
#define BT_DISCONNECT_TIMEOUT   3000    // 断开连接超时（ms），超过此时间未收到数据认为断开

/* ---- 工作模式 ---- */
#define CAR_MODE_MANUAL         0       // 手动模式（手机APP控制）
#define CAR_MODE_TRACKING       1       // 循迹模式（自动循迹）
#define CAR_MODE_OBSTACLE       2       // 避障模式（自动避障）

/* ---- 模式切换命令（APP发送4个相同字节） ---- */
#define MODE_CMD_TRACKING       0x11    // 切换到循迹模式
#define MODE_CMD_OBSTACLE       0x22    // 切换到避障模式
#define MODE_CMD_MANUAL         0x33    // 切换到手动模式

/*============================================================================*/
/*                              数据结构定义                                   */
/*============================================================================*/

/*
 * BT_Command - 蓝牙命令结构体
 * 存储从手机APP接收到的方向和速度命令
 */
typedef struct {
    uint8_t direction;      // 方向命令（0x00~0x08）
    uint8_t speed;          // 速度值（0~100）
    uint8_t is_valid;       // 命令是否有效（1=有效，0=无效）
    uint32_t timestamp;     // 命令接收时间戳（ms）
} BT_Command;

/*
 * BT_State - 蓝牙连接状态结构体
 * 记录蓝牙的连接状态和接收数据统计
 */
typedef struct {
    uint8_t state;          // 连接状态（BT_STATE_CONNECTED / BT_STATE_DISCONNECTED）
    uint32_t last_rx_time;  // 最后一次收到数据的时间戳（ms）
    uint16_t rx_count;      // 总共接收到的字节数
} BT_State;

/* 全局变量声明（在Bluetooth.c中定义） */
extern BT_Command g_bt_cmd;                 // 当前蓝牙命令
extern BT_State g_bt_state;                 // 蓝牙连接状态
extern volatile uint8_t g_car_mode;         // 当前工作模式（手动/循迹/避障）
extern volatile uint8_t g_mode_changed;     // 模式改变标志（1=模式刚被切换）

/*============================================================================*/
/*                              函数声明                                       */
/*============================================================================*/

void Bluetooth_Init(void);                              // 初始化蓝牙模块（USART1 + SysTick）
void Bluetooth_InitNVIC(void);                          // 初始化USART1中断优先级
void Bluetooth_SysTick_Handler(void);                   // SysTick中断处理函数（1ms递增一次）
void Bluetooth_ProcessReceivedData(uint8_t data);       // 处理接收到的蓝牙数据
uint8_t Bluetooth_GetCommand(BT_Command *cmd);          // 获取蓝牙命令（非阻塞）
void Bluetooth_ClearCommand(void);                      // 清除当前命令
uint8_t Bluetooth_IsConnected(void);                    // 检查蓝牙是否连接
uint32_t Bluetooth_GetLastRxTime(void);                 // 获取最后接收时间
uint16_t Bluetooth_GetRxCount(void);                    // 获取接收字节数
void Bluetooth_ExecuteCommand(BT_Command *cmd);         // 执行蓝牙命令（控制电机）
uint32_t Bluetooth_GetTick(void);                       // 获取系统滴答计数（ms）

#endif
