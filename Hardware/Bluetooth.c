/*
 * Bluetooth.c - 蓝牙通信模块
 * 
 * 本模块负责通过HC-06蓝牙模块与手机APP通信
 * 使用USART1（PA9=TX, PA10=RX），波特率9600
 * 
 * 功能：
 *   1. 接收手机APP发送的方向控制命令
 *   2. 接收手机APP发送的速度控制命令
 *   3. 检测模式切换命令（手动/循迹/避障）
 *   4. 检测蓝牙连接/断开状态
 *   5. 执行蓝牙命令控制电机
 * 
 * 通信协议详见Bluetooth.h
 */

#include "Bluetooth.h"
#include "Motor.h"
#include "../System/Delay.h"

/* 全局变量定义 */
BT_Command g_bt_cmd = {0};              // 当前蓝牙命令
BT_State g_bt_state = {0};              // 蓝牙连接状态
volatile uint8_t g_car_mode = CAR_MODE_MANUAL;   // 当前工作模式，默认为手动模式
volatile uint8_t g_mode_changed = 0;             // 模式改变标志

/* 内部变量 */
static uint8_t rx_buffer[BT_RX_BUFFER_SIZE];            // 接收环形缓冲区
static volatile uint16_t rx_head = 0;                   // 缓冲区写指针
static uint8_t speed_buf[BT_CMD_BUFFER_SIZE];           // 速度命令解析缓冲区
static uint8_t speed_idx = 0, speed_active = 0;         // 速度解析状态
static volatile uint32_t g_sys_tick_ms = 0;             // 系统滴答计数（1ms递增）

/* 模式命令检测：连续接收4个相同字节 */
static uint8_t mode_buf[4];     // 模式命令缓冲区
static uint8_t mode_idx = 0;    // 模式命令缓冲区索引

/* 内部函数声明 */
static void ParseDirection(uint8_t data);       // 解析方向命令
static void ParseSpeed(uint8_t data);           // 解析速度命令
static void CheckModeCmd(uint8_t data);         // 检测模式切换命令

/*============================================================================*/
/*                              SysTick系统滴答                                 */
/*============================================================================*/

/*
 * SysTick_Init - 初始化SysTick定时器
 * 配置SysTick每1ms产生一次中断
 * 用于提供系统时间戳和超时判断
 */
static void SysTick_Init(void)
{
    /*
     * SysTick频率 = 系统时钟 / 1000 = 72MHz / 1000 = 72kHz
     * 即每1ms中断一次
     */
    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);  // 配置失败则死循环
    NVIC_SetPriority(SysTick_IRQn, 0x0F);  // 设置SysTick中断优先级为15（最低）
}

/*
 * Bluetooth_SysTick_Handler - SysTick中断处理函数
 * 每1ms调用一次，递增系统滴答计数
 * 在stm32f10x_it.c的SysTick_Handler中调用
 */
void Bluetooth_SysTick_Handler(void) { g_sys_tick_ms++; }

/*
 * Bluetooth_GetTick - 获取当前系统滴答计数（ms）
 * 使用do-while循环确保读取的数值一致（防止中断中修改导致读取到错误值）
 * 返回值：从系统启动到现在的毫秒数
 */
uint32_t Bluetooth_GetTick(void)
{
    uint32_t tick;
    do { tick = g_sys_tick_ms; } while (tick != g_sys_tick_ms);
    return tick;
}

/*============================================================================*/
/*                              初始化函数                                     */
/*============================================================================*/

/*
 * Bluetooth_Init - 蓝牙模块初始化
 * 1. 初始化SysTick定时器（提供时间基准）
 * 2. 配置USART1的TX(PA9)为复用推挽输出
 * 3. 配置USART1的RX(PA10)为浮空输入
 * 4. 配置USART1参数：9600波特率，8位数据，1位停止位，无校验
 * 5. 使能USART1接收中断
 * 6. 初始化所有状态变量
 */
void Bluetooth_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    SysTick_Init();     // 初始化系统滴答

    /* 使能GPIOA和USART1时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* TX(PA9) - 复用推挽输出 */
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* RX(PA10) - 浮空输入 */
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* USART1配置：9600-8-N-1 */
    usart.USART_BaudRate = 9600;                    // 波特率9600
    usart.USART_WordLength = USART_WordLength_8b;   // 8位数据
    usart.USART_StopBits = USART_StopBits_1;        // 1位停止位
    usart.USART_Parity = USART_Parity_No;           // 无校验
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件流控
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  // 使能发送和接收
    USART_Init(USART1, &usart);

    /* 使能接收中断（RXNE：接收非空） */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    Bluetooth_InitNVIC();       // 配置中断优先级
    USART_Cmd(USART1, ENABLE);  // 使能USART1

    /* 初始化状态变量 */
    g_bt_state.state = BT_STATE_DISCONNECTED;   // 初始为断开状态
    g_bt_state.last_rx_time = 0;                // 最后接收时间清零
    g_bt_state.rx_count = 0;                    // 接收计数清零
    g_bt_cmd.direction = BT_CMD_STOP;           // 初始方向为停止
    g_bt_cmd.speed = 50;                        // 默认速度50
    g_bt_cmd.is_valid = 0;                      // 命令无效
    g_bt_cmd.timestamp = 0;                     // 时间戳清零
    speed_idx = 0;
    speed_active = 0;
    mode_idx = 0;
    g_car_mode = CAR_MODE_MANUAL;               // 默认手动模式
    g_mode_changed = 0;
}

/*
 * Bluetooth_InitNVIC - 初始化USART1中断优先级
 * 设置NVIC优先级分组为2位抢占优先级+2位响应优先级
 * USART1中断：抢占优先级1，响应优先级1
 */
void Bluetooth_InitNVIC(void)
{
    NVIC_InitTypeDef nvic;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     // 2位抢占+2位响应
    nvic.NVIC_IRQChannel = USART1_IRQn;                 // USART1中断
    nvic.NVIC_IRQChannelPreemptionPriority = 1;         // 抢占优先级1
    nvic.NVIC_IRQChannelSubPriority = 1;                // 响应优先级1
    nvic.NVIC_IRQChannelCmd = ENABLE;                   // 使能中断
    NVIC_Init(&nvic);
}

/*============================================================================*/
/*                              数据接收与解析                                 */
/*============================================================================*/

/*
 * Bluetooth_ProcessReceivedData - 处理接收到的蓝牙数据
 * 在USART1中断中被调用
 * 
 * 处理流程：
 *   1. 更新连接状态和时间戳
 *   2. 将数据存入环形缓冲区
 *   3. 检测是否为模式切换命令
 *   4. 如果是速度命令（以0x50开头），进入速度解析模式
 *   5. 否则解析为方向命令
 * 
 * data: 接收到的字节
 */
void Bluetooth_ProcessReceivedData(uint8_t data)
{
    /* 更新连接状态 */
    g_bt_state.last_rx_time = Bluetooth_GetTick();  // 记录接收时间
    g_bt_state.rx_count++;                          // 接收计数+1
    g_bt_state.state = BT_STATE_CONNECTED;          // 标记为已连接

    /* 存入环形缓冲区 */
    rx_buffer[rx_head] = data;
    rx_head = (rx_head + 1) % BT_RX_BUFFER_SIZE;

    /* 先检测是否为模式切换命令 */
    CheckModeCmd(data);
    if (g_mode_changed) return;  // 模式已切换，不再解析方向/速度

    /* 判断是速度命令还是方向命令 */
    if (data == BT_SPEED_HEADER) {
        speed_active = 1;       // 进入速度解析模式
        speed_idx = 0;          // 清空速度缓冲区
        return;
    }
    if (speed_active)
        ParseSpeed(data);       // 解析速度命令
    else
        ParseDirection(data);   // 解析方向命令
}

/*
 * CheckModeCmd - 检测模式切换命令
 * 
 * 模式切换采用"连续4个相同字节"的方式，防止误触发
 * 比如APP发送 0x11 0x11 0x11 0x11 就切换到循迹模式
 * 
 * data: 接收到的字节
 */
static void CheckModeCmd(uint8_t data)
{
    /* 只检测0x11/0x22/0x33这三个值 */
    if (data != MODE_CMD_TRACKING && data != MODE_CMD_OBSTACLE && data != MODE_CMD_MANUAL) {
        mode_idx = 0;           // 不是模式命令，重置
        return;
    }

    /* 存入缓冲区 */
    if (mode_idx < 4)
        mode_buf[mode_idx++] = data;
    else {
        /* 缓冲区已满，移位：丢弃最早的一个 */
        for (uint8_t i = 0; i < 3; i++)
            mode_buf[i] = mode_buf[i + 1];
        mode_buf[3] = data;
    }

    /* 检查是否4个字节都相同 */
    if (mode_idx >= 4 &&
        mode_buf[0] == mode_buf[1] &&
        mode_buf[1] == mode_buf[2] &&
        mode_buf[2] == mode_buf[3])
    {
        uint8_t new_mode;
        if (data == MODE_CMD_TRACKING)      new_mode = CAR_MODE_TRACKING;
        else if (data == MODE_CMD_OBSTACLE) new_mode = CAR_MODE_OBSTACLE;
        else                                new_mode = CAR_MODE_MANUAL;

        if (new_mode != g_car_mode) {
            g_car_mode = new_mode;      // 切换模式
            g_mode_changed = 1;         // 设置模式改变标志
            Car_Stop();                 // 切换模式时先停车
        }
        mode_idx = 0;  // 重置，防止重复触发
    }
}

/*
 * ParseDirection - 解析方向命令
 * 如果数据在0x00~0x08范围内，则更新方向命令
 * data: 接收到的字节
 */
static void ParseDirection(uint8_t data)
{
    if (data <= BT_CMD_FORWARD_LEFT) {
        g_bt_cmd.direction = data;          // 更新方向
        g_bt_cmd.is_valid = 1;              // 标记命令有效
        g_bt_cmd.timestamp = Bluetooth_GetTick();  // 记录时间戳
    }
}

/*
 * ParseSpeed - 解析速度命令
 * 
 * 速度命令格式：0x50 [速度值字节...] 0x59
 * 最后一个字节作为实际速度值
 * 比如：0x50 0x3C 0x59 → 速度=60
 * 
 * data: 接收到的字节
 */
static void ParseSpeed(uint8_t data)
{
    if (data == BT_SPEED_TAIL) {
        /* 收到命令尾，解析速度值 */
        if (speed_idx >= 4) {
            /* 有多个速度值，取最后一个 */
            uint8_t v = speed_buf[speed_idx - 1];
            if (v <= BT_SPEED_MAX) {
                g_bt_cmd.speed = v;
                g_bt_cmd.is_valid = 1;
                g_bt_cmd.timestamp = Bluetooth_GetTick();
            }
        } else if (speed_idx >= 1) {
            /* 只有一个速度值 */
            uint8_t v = speed_buf[speed_idx - 1];
            if (v <= BT_SPEED_MAX) {
                g_bt_cmd.speed = v;
                g_bt_cmd.is_valid = 1;
                g_bt_cmd.timestamp = Bluetooth_GetTick();
            }
        }
        speed_active = 0;   // 退出速度解析模式
        speed_idx = 0;      // 清空缓冲区
    } else {
        /* 收到速度值，存入缓冲区 */
        if (speed_idx < BT_CMD_BUFFER_SIZE)
            speed_buf[speed_idx++] = data;
        else {
            /* 缓冲区溢出，放弃本次速度命令 */
            speed_active = 0;
            speed_idx = 0;
        }
    }
}

/*============================================================================*/
/*                              命令获取与状态查询                             */
/*============================================================================*/

/*
 * Bluetooth_GetCommand - 获取蓝牙命令（非阻塞）
 * 如果有有效命令，复制到cmd结构体中并返回1
 * 否则返回0
 * 
 * cmd: 用于接收命令的结构体指针
 * 返回值：1=有命令，0=无命令
 */
uint8_t Bluetooth_GetCommand(BT_Command *cmd)
{
    if (g_bt_cmd.is_valid) {
        cmd->direction = g_bt_cmd.direction;
        cmd->speed = g_bt_cmd.speed;
        cmd->is_valid = 1;
        cmd->timestamp = g_bt_cmd.timestamp;
        g_bt_cmd.is_valid = 0;  // 清除命令（已读取）
        return 1;
    }
    return 0;
}

/*
 * Bluetooth_ClearCommand - 清除当前命令
 */
void Bluetooth_ClearCommand(void)
{
    g_bt_cmd.is_valid = 0;
    g_bt_cmd.direction = BT_CMD_STOP;
}

/*
 * Bluetooth_IsConnected - 检查蓝牙是否连接
 * 
 * 判断逻辑：
 *   如果距离最后一次收到数据超过3秒（BT_DISCONNECT_TIMEOUT），
 *   则认为蓝牙已断开
 * 
 * 返回值：1=已连接，0=未连接
 */
uint8_t Bluetooth_IsConnected(void)
{
    uint32_t now = Bluetooth_GetTick();
    if (now - g_bt_state.last_rx_time > BT_DISCONNECT_TIMEOUT) {
        g_bt_state.state = BT_STATE_DISCONNECTED;  // 超时，标记为断开
        return 0;
    }
    return (g_bt_state.state == BT_STATE_CONNECTED);
}

uint32_t Bluetooth_GetLastRxTime(void) { return g_bt_state.last_rx_time; }
uint16_t Bluetooth_GetRxCount(void) { return g_bt_state.rx_count; }

/*============================================================================*/
/*                              命令执行                                       */
/*============================================================================*/

/* 记录上一次的方向，用于检测正反转切换 */
static uint8_t g_last_dir = BT_CMD_STOP;
/* 转弯速度上限，防止转弯时速度过快翻车 */
#define TURN_SPEED_LIMIT 100

/*
 * IsFwdBwdSwitch - 检测是否发生了正反转切换
 * 比如从前进突然切到后退，或者从后退突然切到前进
 * 这种情况下需要先停车再启动，防止电流冲击损坏电机
 * 
 * old: 旧方向
 * new: 新方向
 * 返回值：1=发生了正反转切换，0=没有
 */
static uint8_t IsFwdBwdSwitch(uint8_t old, uint8_t new)
{
    if ((old == BT_CMD_FORWARD && new == BT_CMD_BACKWARD) ||
        (old == BT_CMD_BACKWARD && new == BT_CMD_FORWARD))
        return 1;
    if ((old == BT_CMD_FORWARD && new == BT_CMD_BACKWARD_RIGHT) ||
        (old == BT_CMD_BACKWARD_RIGHT && new == BT_CMD_FORWARD))
        return 1;
    if ((old == BT_CMD_FORWARD && new == BT_CMD_BACKWARD_LEFT) ||
        (old == BT_CMD_BACKWARD_LEFT && new == BT_CMD_FORWARD))
        return 1;
    if ((old == BT_CMD_BACKWARD && new == BT_CMD_FORWARD_RIGHT) ||
        (old == BT_CMD_FORWARD_RIGHT && new == BT_CMD_BACKWARD))
        return 1;
    if ((old == BT_CMD_BACKWARD && new == BT_CMD_FORWARD_LEFT) ||
        (old == BT_CMD_FORWARD_LEFT && new == BT_CMD_BACKWARD))
        return 1;
    return 0;
}

/*
 * IsTurn - 判断是否为转弯命令
 * 转弯时速度需要限制，防止翻车
 */
static uint8_t IsTurn(uint8_t dir)
{
    switch (dir) {
        case BT_CMD_LEFT: case BT_CMD_RIGHT:
        case BT_CMD_FORWARD_LEFT: case BT_CMD_FORWARD_RIGHT:
        case BT_CMD_BACKWARD_LEFT: case BT_CMD_BACKWARD_RIGHT:
            return 1;
        default: return 0;
    }
}

/*
 * Limit - 限幅函数
 * 如果s超过max，返回max，否则返回s
 */
static uint8_t Limit(uint8_t s, uint8_t max)
{
    return (s > max) ? max : s;
}

/*
 * Bluetooth_ExecuteCommand - 执行蓝牙命令
 * 
 * 根据命令中的方向和速度控制电机
 * 如果是转弯命令，速度会被限制在TURN_SPEED_LIMIT以内
 * 如果检测到正反转切换，会先停车500ms再启动
 * 
 * cmd: 要执行的命令
 */
void Bluetooth_ExecuteCommand(BT_Command *cmd)
{
    if (!cmd->is_valid) return;

    uint8_t speed = cmd->speed;
    if (IsTurn(cmd->direction))
        speed = Limit(speed, TURN_SPEED_LIMIT);  // 转弯限速

    /* 检测正反转切换，需要先停车再启动 */
    if (g_last_dir != BT_CMD_STOP && cmd->direction != BT_CMD_STOP &&
        cmd->direction != g_last_dir && IsFwdBwdSwitch(g_last_dir, cmd->direction))
    {
        g_last_dir = BT_CMD_STOP;
        Motor_SetSpeed(MOTOR_LF, 0);
        Motor_SetSpeed(MOTOR_LB, 0);
        Motor_SetSpeed(MOTOR_RF, 0);
        Motor_SetSpeed(MOTOR_RB, 0);
        
    }

    g_last_dir = cmd->direction;

    /* 根据方向命令执行对应的运动 */
    switch (cmd->direction) {
        case BT_CMD_STOP:           Car_Stop(); break;
        case BT_CMD_FORWARD:        Car_Forward(speed); break;
        case BT_CMD_FORWARD_RIGHT:  Car_ForwardTurnRight(speed); break;
        case BT_CMD_RIGHT:          Car_TurnRight(speed); break;
        case BT_CMD_BACKWARD_RIGHT: Car_BackwardTurnRight(speed); break;
        case BT_CMD_BACKWARD:       Car_Backward(speed); break;
        case BT_CMD_BACKWARD_LEFT:  Car_BackwardTurnLeft(speed); break;
        case BT_CMD_LEFT:           Car_TurnLeft(speed); break;
        case BT_CMD_FORWARD_LEFT:   Car_ForwardTurnLeft(speed); break;
        default:                    Car_Stop(); break;
    }
}
