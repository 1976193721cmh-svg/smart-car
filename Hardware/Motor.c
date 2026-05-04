/*
 * Motor.c - 电机驱动模块
 * 
 * 本模块负责控制小车的4个直流电机（左前、左后、右前、右后）
 * 使用2个TB6612驱动芯片，每个芯片控制一侧的2个电机
 * 
 * 控制原理：
 *   每个电机有3个控制信号：
 *     1. PWM（脉冲宽度调制）：控制速度，占空比越大速度越快
 *     2. IN1/IN2（方向控制）：控制正转/反转/停止
 *        IN1=1, IN2=0 → 正转（前进）
 *        IN1=0, IN2=1 → 反转（后退）
 *        IN1=0, IN2=0 → 停止（刹车）
 * 
 * 引脚分配：
 *   TB6612 #1（左轮）：TIM2_CH1(PA0), TIM2_CH4(PA3)
 *   TB6612 #2（右轮）：TIM3_CH3(PB0), TIM3_CH1(PA6)
 */

#include "Motor.h"
#include "../System/Delay.h"

/*
 * 整车状态变量（静态，外部通过函数访问）
 * 初始化所有电机为停止状态
 */
static Car_State car_state = {
    .motor_lf = {0, MOTOR_DIR_STOP, 0},    // 左前轮：速度0，停止，未运行
    .motor_lb = {0, MOTOR_DIR_STOP, 0},    // 左后轮：速度0，停止，未运行
    .motor_rf = {0, MOTOR_DIR_STOP, 0},    // 右前轮：速度0，停止，未运行
    .motor_rb = {0, MOTOR_DIR_STOP, 0},    // 右后轮：速度0，停止，未运行
    .car_speed = 0,                         // 整车速度：0
    .car_direction = 0                      // 整车方向：0（停止）
};

/* 内部函数声明 */
static void Motor_GPIO_Init(void);          // 初始化电机方向控制引脚
static void Motor_PWM_Init(void);           // 初始化PWM定时器
static void SetSingleDir(Motor_ID motor, Motor_Direction dir);   // 设置单个电机方向（硬件操作）
static void SetSinglePWM(Motor_ID motor, uint8_t duty);          // 设置单个电机PWM占空比

/*============================================================================*/
/*                              初始化函数                                     */
/*============================================================================*/

/*
 * Motor_Init - 电机初始化
 * 1. 禁用JTAG（释放PB3/PB4给电机使用）
 * 2. 初始化GPIO（方向控制引脚）
 * 3. 初始化PWM（速度控制引脚）
 * 4. 停止所有电机
 */
void Motor_Init(void)
{
    /* 使能AFIO时钟，用于重映射功能 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /* 禁用JTAG，释放PB3(JTDO)/PB4(JNTRST)给普通GPIO使用 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    Motor_GPIO_Init();      // 初始化方向控制引脚
    Motor_PWM_Init();       // 初始化PWM定时器
    Motor_StopAll();        // 确保所有电机初始为停止状态
    car_state.car_speed = 0;
    car_state.car_direction = 0;
}

/*============================================================================*/
/*                              基本电机控制                                   */
/*============================================================================*/

/*
 * Motor_SetSpeed - 设置单个电机的速度
 * motor: 电机编号（MOTOR_LF/LB/RF/RB）
 * speed: 速度值（0~100），超过100会自动限制为100
 */
void Motor_SetSpeed(Motor_ID motor, uint8_t speed)
{
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;  // 限幅
    SetSinglePWM(motor, speed);     // 设置硬件PWM
    
    /* 更新状态变量 */
    Motor_State *s;
    switch (motor) {
        case MOTOR_LF: s = &car_state.motor_lf; break;
        case MOTOR_LB: s = &car_state.motor_lb; break;
        case MOTOR_RF: s = &car_state.motor_rf; break;
        case MOTOR_RB: s = &car_state.motor_rb; break;
        default: return;
    }
    s->speed = speed;
    s->is_running = (speed > 0);    // 速度>0表示正在运行
}

/*
 * Motor_SetDirection - 设置单个电机的方向
 * motor: 电机编号
 * dir: 方向（MOTOR_DIR_FORWARD/BACKWARD/STOP）
 */
void Motor_SetDirection(Motor_ID motor, Motor_Direction dir)
{
    SetSingleDir(motor, dir);       // 设置硬件方向
    
    /* 更新状态变量 */
    Motor_State *s;
    switch (motor) {
        case MOTOR_LF: s = &car_state.motor_lf; break;
        case MOTOR_LB: s = &car_state.motor_lb; break;
        case MOTOR_RF: s = &car_state.motor_rf; break;
        case MOTOR_RB: s = &car_state.motor_rb; break;
        default: return;
    }
    s->dir = dir;
}

/*
 * Motor_Stop - 停止单个电机
 * 将速度设为0，方向设为停止
 */
void Motor_Stop(Motor_ID motor)
{
    Motor_SetSpeed(motor, 0);               // 速度归零
    Motor_SetDirection(motor, MOTOR_DIR_STOP);  // 方向设为停止
    Motor_State *s;
    switch (motor) {
        case MOTOR_LF: s = &car_state.motor_lf; break;
        case MOTOR_LB: s = &car_state.motor_lb; break;
        case MOTOR_RF: s = &car_state.motor_rf; break;
        case MOTOR_RB: s = &car_state.motor_rb; break;
        default: return;
    }
    s->is_running = 0;  // 标记为未运行
}

/*
 * Motor_StopAll - 停止所有电机
 * 紧急停车时调用
 */
void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_LF); Motor_Stop(MOTOR_LB);
    Motor_Stop(MOTOR_RF); Motor_Stop(MOTOR_RB);
    car_state.car_speed = 0;
    car_state.car_direction = 0;
}

/*============================================================================*/
/*                              轮组控制                                       */
/*============================================================================*/

/*
 * Motor_SetWheelSpeed - 设置一侧轮子的速度
 * wheel: WHEEL_LEFT（左轮组）或 WHEEL_RIGHT（右轮组）
 * speed: 速度值（0~100）
 * 左轮组包含左前+左后，右轮组包含右前+右后
 */
void Motor_SetWheelSpeed(Wheel_Group wheel, uint8_t speed)
{
    if (speed > MOTOR_SPEED_MAX) speed = MOTOR_SPEED_MAX;
    if (wheel == WHEEL_LEFT) {
        SetSinglePWM(MOTOR_LF, speed); SetSinglePWM(MOTOR_LB, speed);
        car_state.motor_lf.speed = speed; car_state.motor_lb.speed = speed;
        car_state.motor_lf.is_running = (speed > 0);
        car_state.motor_lb.is_running = (speed > 0);
    } else {
        SetSinglePWM(MOTOR_RF, speed); SetSinglePWM(MOTOR_RB, speed);
        car_state.motor_rf.speed = speed; car_state.motor_rb.speed = speed;
        car_state.motor_rf.is_running = (speed > 0);
        car_state.motor_rb.is_running = (speed > 0);
    }
}

/*
 * Motor_SetWheelDirection - 设置一侧轮子的方向
 */
void Motor_SetWheelDirection(Wheel_Group wheel, Motor_Direction dir)
{
    if (wheel == WHEEL_LEFT) {
        SetSingleDir(MOTOR_LF, dir); SetSingleDir(MOTOR_LB, dir);
        car_state.motor_lf.dir = dir; car_state.motor_lb.dir = dir;
    } else {
        SetSingleDir(MOTOR_RF, dir); SetSingleDir(MOTOR_RB, dir);
        car_state.motor_rf.dir = dir; car_state.motor_rb.dir = dir;
    }
}

/*
 * Motor_StopWheel - 停止一侧轮子
 */
void Motor_StopWheel(Wheel_Group wheel)
{
    if (wheel == WHEEL_LEFT) {
        Motor_Stop(MOTOR_LF); Motor_Stop(MOTOR_LB);
    } else {
        Motor_Stop(MOTOR_RF); Motor_Stop(MOTOR_RB);
    }
}

/*============================================================================*/
/*                              高级运动控制                                   */
/*============================================================================*/

/*
 * 运动控制说明：
 * 
 * 差速转向原理：
 *   小车通过左右轮的速度差来实现转向
 *   就像坦克一样，左右轮速度不同就会转弯
 * 
 * 8种运动模式：
 *   1. 前进：4个轮子都正转
 *   2. 后退：4个轮子都反转
 *   3. 原地左转：左轮反转，右轮正转（像坦克原地掉头）
 *   4. 原地右转：右轮反转，左轮正转
 *   5. 前进左转：左轮减速，右轮全速（大弧线左转）
 *   6. 前进右转：右轮减速，左轮全速（大弧线右转）
 *   7. 后退左转：左轮减速后退，右轮全速后退
 *   8. 后退右转：右轮减速后退，左轮全速后退
 */

void Car_Forward(uint8_t speed)
{
    /* 4个轮子都正转 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_FORWARD);
    /* 4个轮子速度相同 */
    Motor_SetSpeed(MOTOR_LF, speed); Motor_SetSpeed(MOTOR_LB, speed);
    Motor_SetSpeed(MOTOR_RF, speed); Motor_SetSpeed(MOTOR_RB, speed);
    car_state.car_speed = speed;
    car_state.car_direction = 1;    // 方向编码：1=前进
}

void Car_Backward(uint8_t speed)
{
    /* 4个轮子都反转 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_BACKWARD);
    Motor_SetSpeed(MOTOR_LF, speed); Motor_SetSpeed(MOTOR_LB, speed);
    Motor_SetSpeed(MOTOR_RF, speed); Motor_SetSpeed(MOTOR_RB, speed);
    car_state.car_speed = speed;
    car_state.car_direction = 2;    // 方向编码：2=后退
}

void Car_TurnLeft(uint8_t speed)
{
    /* 原地左转：左轮后退，右轮前进 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_FORWARD);
    Motor_SetSpeed(MOTOR_LF, speed); Motor_SetSpeed(MOTOR_LB, speed);
    Motor_SetSpeed(MOTOR_RF, speed); Motor_SetSpeed(MOTOR_RB, speed);
    car_state.car_speed = speed;
    car_state.car_direction = 3;    // 方向编码：3=原地左转
}

void Car_TurnRight(uint8_t speed)
{
    /* 原地右转：右轮后退，左轮前进 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_BACKWARD);
    Motor_SetSpeed(MOTOR_LF, speed); Motor_SetSpeed(MOTOR_LB, speed);
    Motor_SetSpeed(MOTOR_RF, speed); Motor_SetSpeed(MOTOR_RB, speed);
    car_state.car_speed = speed;
    car_state.car_direction = 4;    // 方向编码：4=原地右转
}

void Car_ForwardTurnLeft(uint8_t speed)
{
    /* 前进中左转：左轮减速，右轮全速 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_FORWARD);
    uint8_t ls = (uint8_t)(speed * TURN_SPEED_RATIO);  // 左轮速度 = 右轮速度 × 比例
    Motor_SetSpeed(MOTOR_LF, ls); Motor_SetSpeed(MOTOR_LB, ls);
    Motor_SetSpeed(MOTOR_RF, speed); Motor_SetSpeed(MOTOR_RB, speed);
    car_state.car_speed = speed;
    car_state.car_direction = 5;    // 方向编码：5=前进左转
}

void Car_ForwardTurnRight(uint8_t speed)
{
    /* 前进中右转：右轮减速，左轮全速 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_FORWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_FORWARD);
    uint8_t rs = (uint8_t)(speed * TURN_SPEED_RATIO);  // 右轮速度 = 左轮速度 × 比例
    Motor_SetSpeed(MOTOR_LF, speed); Motor_SetSpeed(MOTOR_LB, speed);
    Motor_SetSpeed(MOTOR_RF, rs); Motor_SetSpeed(MOTOR_RB, rs);
    car_state.car_speed = speed;
    car_state.car_direction = 6;    // 方向编码：6=前进右转
}

void Car_BackwardTurnLeft(uint8_t speed)
{
    /* 后退中左转 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_BACKWARD);
    uint8_t ls = (uint8_t)(speed * TURN_SPEED_RATIO);
    Motor_SetSpeed(MOTOR_LF, ls); Motor_SetSpeed(MOTOR_LB, ls);
    Motor_SetSpeed(MOTOR_RF, speed); Motor_SetSpeed(MOTOR_RB, speed);
    car_state.car_speed = speed;
    car_state.car_direction = 7;    // 方向编码：7=后退左转
}

void Car_BackwardTurnRight(uint8_t speed)
{
    /* 后退中右转 */
    Motor_SetDirection(MOTOR_LF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_LB, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RF, MOTOR_DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RB, MOTOR_DIR_BACKWARD);
    uint8_t rs = (uint8_t)(speed * TURN_SPEED_RATIO);
    Motor_SetSpeed(MOTOR_LF, speed); Motor_SetSpeed(MOTOR_LB, speed);
    Motor_SetSpeed(MOTOR_RF, rs); Motor_SetSpeed(MOTOR_RB, rs);
    car_state.car_speed = speed;
    car_state.car_direction = 8;    // 方向编码：8=后退右转
}

void Car_Stop(void) { Motor_StopAll(); }

/*
 * Car_SetSpeed - 保持当前方向，只改变速度
 * 比如正在前进中，调用此函数可以改变前进的速度
 */
void Car_SetSpeed(uint8_t speed)
{
    switch (car_state.car_direction) {
        case 1: Car_Forward(speed); break;
        case 2: Car_Backward(speed); break;
        case 3: Car_TurnLeft(speed); break;
        case 4: Car_TurnRight(speed); break;
        case 5: Car_ForwardTurnLeft(speed); break;
        case 6: Car_ForwardTurnRight(speed); break;
        case 7: Car_BackwardTurnLeft(speed); break;
        case 8: Car_BackwardTurnRight(speed); break;
        default: car_state.car_speed = speed; break;
    }
}

/*
 * Car_ChangeSpeed - 在当前速度基础上增加或减少
 * delta: 正数加速，负数减速
 * 比如当前速度50，调用Car_ChangeSpeed(10) → 速度变为60
 */
void Car_ChangeSpeed(int8_t delta)
{
    int16_t ns = (int16_t)car_state.car_speed + delta;
    if (ns < MOTOR_SPEED_MIN) ns = MOTOR_SPEED_MIN;    // 不低于最小值
    if (ns > MOTOR_SPEED_MAX) ns = MOTOR_SPEED_MAX;    // 不超过最大值
    Car_SetSpeed((uint8_t)ns);
}

/*============================================================================*/
/*                              状态获取                                       */
/*============================================================================*/

uint8_t Motor_GetSpeed(Motor_ID motor)
{
    switch (motor) {
        case MOTOR_LF: return car_state.motor_lf.speed;
        case MOTOR_LB: return car_state.motor_lb.speed;
        case MOTOR_RF: return car_state.motor_rf.speed;
        case MOTOR_RB: return car_state.motor_rb.speed;
        default: return 0;
    }
}

Motor_Direction Motor_GetDirection(Motor_ID motor)
{
    switch (motor) {
        case MOTOR_LF: return car_state.motor_lf.dir;
        case MOTOR_LB: return car_state.motor_lb.dir;
        case MOTOR_RF: return car_state.motor_rf.dir;
        case MOTOR_RB: return car_state.motor_rb.dir;
        default: return MOTOR_DIR_STOP;
    }
}

uint8_t Motor_GetWheelSpeed(Wheel_Group wheel)
{
    if (wheel == WHEEL_LEFT)
        return (car_state.motor_lf.speed + car_state.motor_lb.speed) / 2;  // 取平均
    else
        return (car_state.motor_rf.speed + car_state.motor_rb.speed) / 2;
}

Motor_Direction Motor_GetWheelDirection(Wheel_Group wheel)
{
    if (wheel == WHEEL_LEFT) return car_state.motor_lf.dir;
    else return car_state.motor_rf.dir;
}

Car_State* Car_GetState(void) { return &car_state; }

/*============================================================================*/
/*                              静态函数（硬件操作）                           */
/*============================================================================*/

/*
 * Motor_GPIO_Init - 初始化电机方向控制引脚
 * 将PA1,PA2,PA4,PA5,PB1,PB3,PB5,PB6配置为推挽输出
 * 这些引脚连接到TB6612的IN1/IN2，控制电机方向
 */
static void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;     // 推挽输出
    gpio.GPIO_Speed = GPIO_Speed_50MHz;    // 50MHz高速

    /* 左前轮方向引脚 */
    gpio.GPIO_Pin = MOTOR_LF_IN1_PIN | MOTOR_LF_IN2_PIN;
    GPIO_Init(MOTOR_LF_IN1_PORT, &gpio);
    /* 左后轮方向引脚 */
    gpio.GPIO_Pin = MOTOR_LB_IN1_PIN | MOTOR_LB_IN2_PIN;
    GPIO_Init(MOTOR_LB_IN1_PORT, &gpio);
    /* 右前轮方向引脚 */
    gpio.GPIO_Pin = MOTOR_RF_IN1_PIN | MOTOR_RF_IN2_PIN;
    GPIO_Init(MOTOR_RF_IN1_PORT, &gpio);
    /* 右后轮方向引脚 */
    gpio.GPIO_Pin = MOTOR_RB_IN1_PIN | MOTOR_RB_IN2_PIN;
    GPIO_Init(MOTOR_RB_IN1_PORT, &gpio);
}

/*
 * Motor_PWM_Init - 初始化PWM定时器
 * 
 * 使用TIM2和TIM3产生PWM信号
 * TIM2：左前轮(PA0_CH1)，左后轮(PA3_CH4)
 * TIM3：右前轮(PB0_CH3)，右后轮(PA6_CH1)
 * 
 * PWM频率计算：
 *   系统时钟 = 72MHz
 *   预分频 = 36-1 → 72MHz/36 = 2MHz
 *   自动重装 = 100-1 → 2MHz/100 = 20kHz
 *   所以PWM频率 = 20kHz（人耳听不到）
 */
static void Motor_PWM_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    /* 使能GPIO和定时器时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 配置PWM引脚为复用推挽输出 */
    gpio.GPIO_Pin = MOTOR_LF_PWM_PIN | MOTOR_LB_PWM_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出（由定时器控制）
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_LF_PWM_PORT, &gpio);

    gpio.GPIO_Pin = MOTOR_RF_PWM_PIN;
    GPIO_Init(MOTOR_RF_PWM_PORT, &gpio);
    gpio.GPIO_Pin = MOTOR_RB_PWM_PIN;
    GPIO_Init(MOTOR_RB_PWM_PORT, &gpio);

    /* 定时器基础配置 */
    tim.TIM_Period = MOTOR_PWM_PERIOD - 1;  // 自动重装值：99（0~99共100个计数）
    tim.TIM_Prescaler = 36 - 1;             // 预分频：35（72MHz/36=2MHz）
    tim.TIM_ClockDivision = 0;              // 时钟不分频
    tim.TIM_CounterMode = TIM_CounterMode_Up;   // 向上计数模式
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_TimeBaseInit(TIM3, &tim);

    /* PWM通道配置 */
    oc.TIM_OCMode = TIM_OCMode_PWM1;       // PWM模式1
    oc.TIM_OutputState = TIM_OutputState_Enable;    // 使能输出
    oc.TIM_OCPolarity = TIM_OCPolarity_High;        // 高电平有效
    oc.TIM_Pulse = 0;                       // 初始占空比为0（停止）

    /* 初始化4个PWM通道 */
    TIM_OC1Init(TIM2, &oc); TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM2, &oc); TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC1Init(TIM3, &oc); TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM3, &oc); TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* 启动定时器 */
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

/*
 * SetSingleDir - 设置单个电机的方向（硬件操作）
 * 通过控制TB6612的IN1和IN2引脚电平来实现
 * 
 * TB6612真值表：
 *   IN1=1, IN2=0 → 正转（前进）
 *   IN1=0, IN2=1 → 反转（后退）
 *   IN1=0, IN2=0 → 停止（刹车）
 */
static void SetSingleDir(Motor_ID motor, Motor_Direction dir)
{
    GPIO_TypeDef *p1, *p2;
    uint16_t pin1, pin2;

    /* 根据电机编号选择对应的GPIO端口和引脚 */
    switch (motor) {
        case MOTOR_LF:
            p1 = MOTOR_LF_IN1_PORT; pin1 = MOTOR_LF_IN1_PIN;
            p2 = MOTOR_LF_IN2_PORT; pin2 = MOTOR_LF_IN2_PIN;
            break;
        case MOTOR_LB:
            p1 = MOTOR_LB_IN1_PORT; pin1 = MOTOR_LB_IN1_PIN;
            p2 = MOTOR_LB_IN2_PORT; pin2 = MOTOR_LB_IN2_PIN;
            break;
        case MOTOR_RF:
            p1 = MOTOR_RF_IN1_PORT; pin1 = MOTOR_RF_IN1_PIN;
            p2 = MOTOR_RF_IN2_PORT; pin2 = MOTOR_RF_IN2_PIN;
            break;
        case MOTOR_RB:
            p1 = MOTOR_RB_IN1_PORT; pin1 = MOTOR_RB_IN1_PIN;
            p2 = MOTOR_RB_IN2_PORT; pin2 = MOTOR_RB_IN2_PIN;
            break;
        default: return;
    }

    /* 根据方向设置IN1/IN2电平 */
    switch (dir) {
        case MOTOR_DIR_FORWARD:
            GPIO_SetBits(p1, pin1); GPIO_ResetBits(p2, pin2); break;  // IN1=1, IN2=0
        case MOTOR_DIR_BACKWARD:
            GPIO_ResetBits(p1, pin1); GPIO_SetBits(p2, pin2); break;  // IN1=0, IN2=1
        case MOTOR_DIR_STOP:
            GPIO_ResetBits(p1, pin1); GPIO_ResetBits(p2, pin2); break; // IN1=0, IN2=0
    }
}

/*
 * SetSinglePWM - 设置单个电机的PWM占空比（硬件操作）
 * 通过修改定时器的比较值来控制占空比
 * duty: 占空比（0~100），0=停止，100=全速
 */
static void SetSinglePWM(Motor_ID motor, uint8_t duty)
{
    if (duty > MOTOR_PWM_PERIOD) duty = MOTOR_PWM_PERIOD;  // 限幅
    switch (motor) {
        case MOTOR_LF: TIM_SetCompare1(TIM2, duty); break;  // TIM2_CH1：左前轮
        case MOTOR_LB: TIM_SetCompare4(TIM2, duty); break;  // TIM2_CH4：左后轮
        case MOTOR_RF: TIM_SetCompare3(TIM3, duty); break;  // TIM3_CH3：右前轮
        case MOTOR_RB: TIM_SetCompare1(TIM3, duty); break;  // TIM3_CH1：右后轮
    }
}
