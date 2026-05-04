/*
 * Servo.c - SG90舵机驱动模块
 * 
 * 本模块负责控制SG90舵机，用于超声波传感器的方向扫描
 * 使用TIM4_CH2（PB7）产生50Hz的PWM信号
 * 
 * 舵机用途：
 *   在避障模式下，舵机带动超声波传感器左右扫描
 *   测量前方、左方、右方的障碍物距离
 *   从而决定往哪个方向避让
 */

#include "Servo.h"
#include "../System/Delay.h"

/*
 * Servo_Init - 初始化舵机PWM
 * 
 * 使用TIM4_CH2（PB7）产生50Hz的PWM信号
 * 
 * 定时器配置计算：
 *   系统时钟 = 72MHz
 *   预分频 = 72-1 → 72MHz/72 = 1MHz（每1us计数一次）
 *   自动重装 = 20000-1 → 1MHz/20000 = 50Hz（周期20ms）
 * 
 * 这样CCR每增加1，高电平就增加1us
 * 所以CCR=499 → 0.5ms → 0度
 *     CCR=1499 → 1.5ms → 90度
 *     CCR=2499 → 2.5ms → 180度
 */
void Servo_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 配置PB7为复用推挽输出（由TIM4_CH2控制） */
    gpio.GPIO_Pin = SERVO_PWM_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_PWM_PORT, &gpio);

    /* TIM4配置：50Hz */
    TIM_DeInit(SERVO_PWM_TIM);
    tim.TIM_Prescaler = 72 - 1;             // 预分频：71（72MHz/72=1MHz）
    tim.TIM_Period = 20000 - 1;             // 自动重装：19999（1MHz/20000=50Hz）
    tim.TIM_CounterMode = TIM_CounterMode_Up;   // 向上计数模式
    tim.TIM_ClockDivision = TIM_CKD_DIV1;       // 时钟不分频
    TIM_TimeBaseInit(SERVO_PWM_TIM, &tim);

    /* TIM4_CH2 PWM模式1 */
    oc.TIM_OCMode = TIM_OCMode_PWM1;       // PWM模式1
    oc.TIM_OutputState = TIM_OutputState_Enable;    // 使能输出
    oc.TIM_Pulse = SERVO_CCR_90DEG;         // 默认90度（中间位置）
    oc.TIM_OCPolarity = TIM_OCPolarity_High;        // 高电平有效
    TIM_OC2Init(SERVO_PWM_TIM, &oc);
    TIM_OC2PreloadConfig(SERVO_PWM_TIM, TIM_OCPreload_Enable);

    TIM_Cmd(SERVO_PWM_TIM, ENABLE);         // 启动定时器
}

/*
 * Servo_SetAngle - 设置舵机角度
 * 
 * 将角度（0~180）线性映射到CCR值（499~2499）
 * 然后调用Servo_SetCCR设置PWM占空比
 * 
 * angle: 目标角度（0~180度）
 *   0度 = 最右边
 *   90度 = 正前方
 *   180度 = 最左边
 */
void Servo_SetAngle(uint8_t angle)
{
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;  // 限幅

    // 线性插值：CCR = 499 + angle * (2499-499)/180
    uint16_t ccr = SERVO_CCR_0DEG + (uint16_t)((uint32_t)angle * 2000 / 180);
    Servo_SetCCR(ccr);
}

/*
 * Servo_SetCCR - 直接设置CCR值
 * 
 * 直接修改TIM4_CH2的比较值来控制PWM占空比
 * 从而控制舵机角度
 * 
 * ccr: 比较值（499~2499）
 *   499 = 0度（最右边）
 *   1499 = 90度（正前方）
 *   2499 = 180度（最左边）
 */
void Servo_SetCCR(uint16_t ccr)
{
    if (ccr < SERVO_CCR_0DEG) ccr = SERVO_CCR_0DEG;        // 下限限幅
    if (ccr > SERVO_CCR_180DEG) ccr = SERVO_CCR_180DEG;    // 上限限幅
    TIM_SetCompare2(SERVO_PWM_TIM, ccr);    // 设置TIM4_CH2的比较值
    Delay_ms(10);  // 给舵机响应时间（舵机转动需要时间）
}
