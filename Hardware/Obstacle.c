/*
 * Obstacle.c - 超声波避障模块
 * 
 * 本模块负责实现小车的自动避障功能
 * 使用HC-SR04超声波模块测量距离
 * 使用SG90舵机带动超声波传感器左右扫描
 * 
 * 避障策略：
 *   1. 直行，同时舵机指向前方，持续检测前方距离
 *   2. 前方有障碍（<15cm）→ 停车，扫描左右距离
 *   3. 哪边距离大就往哪边转
 *   4. 转完后直行，继续检测
 * 
 * 超声波测距原理：
 *   声波在空气中的传播速度约为340m/s
 *   距离 = 时间 × 速度 / 2（来回）
 *   简化公式：距离(cm) = 高电平时间(us) / 58
 */

#include "Obstacle.h"
#include "Servo.h"
#include "Motor.h"
#include "../System/Delay.h"

/* 超声波测距结果全局变量 */
Ultrasonic_Data g_ultrasonic = {0};

/* 避障状态机 */
static Avoid_State g_avoid_state = AVOID_STATE_IDLE;    // 当前状态
static uint32_t g_state_start_time = 0;                 // 状态开始时间
static uint8_t g_scan_phase = 0;                        // 扫描阶段：0=测左边，1=测右边

/*
 * Obstacle_Init - 初始化超声波模块和舵机
 * 
 * 1. 配置TRIG(PB8)为推挽输出
 * 2. 配置ECHO(PA11)为浮空输入
 * 3. 初始化舵机
 * 4. 舵机转到正前方
 * 5. 状态机设为IDLE
 */
void Obstacle_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* TRIG - 推挽输出（用于发送触发脉冲） */
    gpio.GPIO_Pin = ULTRASONIC_TRIG_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;      // 推挽输出
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRASONIC_TRIG_PORT, &gpio);
    GPIO_ResetBits(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);  // 初始低电平

    /* ECHO - 浮空输入（用于接收回波信号） */
    gpio.GPIO_Pin = ULTRASONIC_ECHO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 浮空输入
    GPIO_Init(ULTRASONIC_ECHO_PORT, &gpio);

    /* 初始化舵机 */
    Servo_Init();

    /* 舵机转到正前方 */
    Servo_SetAngle(SERVO_ANGLE_FRONT);
    Delay_ms(200);

    g_avoid_state = AVOID_STATE_IDLE;  // 初始状态：空闲
}

/*
 * Ultrasonic_Measure - 超声波测距（单次测量）
 * 
 * 测量步骤：
 *   1. TRIG拉高15us触发测距
 *   2. 等待ECHO变高（模块开始发送超声波）
 *   3. 测量ECHO高电平持续时间
 *   4. 计算距离 = 高电平时间(us) / 58
 * 
 * 返回值：距离（cm），超时返回999
 *   0~400cm：有效测量值
 *   999：测量超时（可能超出量程或模块未响应）
 */
uint16_t Ultrasonic_Measure(void)
{
    uint16_t distance = 999;
    uint32_t timeout;

    /* 发送15us高电平触发脉冲（要求至少10us） */
    GPIO_SetBits(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    Delay_us(15);
    GPIO_ResetBits(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);

    /* 等待ECHO变高（等待模块响应），最长等待5ms */
    timeout = 0;
    while (!GPIO_ReadInputDataBit(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN)) {
        Delay_us(10);
        if (++timeout > 500) return 999;  // 约5ms超时，模块未响应
    }

    /* 测量ECHO高电平持续时间，最长等待30ms（对应约5m） */
    uint32_t pulse_width = 0;
    while (GPIO_ReadInputDataBit(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN)) {
        Delay_us(10);
        pulse_width += 10;
        if (pulse_width > 30000) {  // 30ms超时（对应约5m距离）
            pulse_width = 30000;
            break;
        }
    }

    /* 计算距离：高电平时间(us) / 58 = 距离(cm) */
    if (pulse_width > 0) {
        distance = pulse_width / 58;
        if (distance > 400) distance = 400;  // 限制最大距离为400cm
    }

    return distance;
}

/*
 * Obstacle_Run - 避障运行函数
 * 
 * 在main循环中周期调用，实现自动避障
 * 
 * 避障流程详解：
 * 
 *   IDLE状态：
 *     舵机指向前方，测量前方距离
 *     如果前方距离 < 阈值(15cm) → 切换到SCAN状态
 *     否则 → 直行
 * 
 *   SCAN状态：
 *     分两步：先测左边，再测右边
 *     测左边：舵机转到180度（最左），测量距离
 *     测右边：舵机转到0度（最右），测量距离
 *     两边都测完后，根据距离决定下一步：
 *       - 左边通畅 → TURN_LEFT
 *       - 右边通畅 → TURN_RIGHT
 *       - 两边都堵 → BACK
 * 
 *   TURN_LEFT状态：
 *     原地左转400ms，然后切换到FORWARD
 * 
 *   TURN_RIGHT状态：
 *     原地右转400ms，然后切换到FORWARD
 * 
 *   BACK状态：
 *     后退500ms，左转300ms，然后切换到FORWARD
 * 
 *   FORWARD状态：
 *     前进500ms，然后回到IDLE继续检测
 */
void Obstacle_Run(void)
{
    uint16_t dist;

    switch (g_avoid_state) {
        case AVOID_STATE_IDLE:
            /* 正前方检测 */
            Servo_SetAngle(SERVO_ANGLE_FRONT);  // 舵机指向前方
            Delay_ms(50);
            dist = Ultrasonic_Measure();        // 测量前方距离
            g_ultrasonic.distance_front = dist;

            if (dist < OBSTACLE_THRESHOLD) {
                /* 前方有障碍，先停车，再扫描 */
                Car_Stop();
                Delay_ms(50);           // 等电机完全停稳
                g_avoid_state = AVOID_STATE_SCAN;  // 切换到扫描状态
                g_scan_phase = 0;       // 从左边开始扫描
                Delay_ms(100);
            } else {
                /* 无障碍，直行 */
                Car_Forward(50);
            }
            break;

        case AVOID_STATE_SCAN:
            /* 扫描左右距离 */
            if (g_scan_phase == 0) {
                /* 测左边 - 分步转动，避免大电流冲击 */
                Car_Stop();             // 确保电机已停
                Servo_SetAngle(135);    // 先转到135度
                Delay_ms(300);
                Servo_SetAngle(SERVO_ANGLE_LEFT);  // 再转到180度（最左）
                Delay_ms(500);
                dist = Ultrasonic_Measure();        // 测量左边距离
                g_ultrasonic.distance_left = dist;
                g_ultrasonic.obstacle_left = (dist < OBSTACLE_THRESHOLD) ? 1 : 0;
                Delay_ms(300);
                g_scan_phase = 1;       // 切换到测右边
            } else {
                /* 测右边 - 分步转动 */
                Car_Stop();
                Servo_SetAngle(45);     // 先转到45度
                Delay_ms(300);
                Servo_SetAngle(SERVO_ANGLE_RIGHT);  // 再转到0度（最右）
                Delay_ms(500);
                dist = Ultrasonic_Measure();        // 测量右边距离
                g_ultrasonic.distance_right = dist;
                g_ultrasonic.obstacle_right = (dist < OBSTACLE_THRESHOLD) ? 1 : 0;

                /* 左右都测完了，决定往哪边转 */
                Servo_SetAngle(SERVO_ANGLE_FRONT);  // 舵机先回正
                Delay_ms(500);                      // 等舵机回正稳定，避免同时大电流

                if (!g_ultrasonic.obstacle_left && !g_ultrasonic.obstacle_right) {
                    /* 两边都通畅，选距离大的方向 */
                    if (g_ultrasonic.distance_left >= g_ultrasonic.distance_right) {
                        g_avoid_state = AVOID_STATE_TURN_LEFT;  // 左边更宽，左转
                    } else {
                        g_avoid_state = AVOID_STATE_TURN_RIGHT; // 右边更宽，右转
                    }
                } else if (!g_ultrasonic.obstacle_left) {
                    /* 左边通畅 */
                    g_avoid_state = AVOID_STATE_TURN_LEFT;
                } else if (!g_ultrasonic.obstacle_right) {
                    /* 右边通畅 */
                    g_avoid_state = AVOID_STATE_TURN_RIGHT;
                } else {
                    /* 两边都有障碍，后退 */
                    g_avoid_state = AVOID_STATE_BACK;
                }

                g_state_start_time = 0;
            }
            break;

        case AVOID_STATE_TURN_LEFT:
            /* 左转避障 */
            Car_TurnLeft(80);
            Delay_ms(400);
            /* 转完后回正，继续前进 */
            g_avoid_state = AVOID_STATE_FORWARD;
            break;

        case AVOID_STATE_TURN_RIGHT:
            /* 右转避障 */
            Car_TurnRight(80);
            Delay_ms(400);
            g_avoid_state = AVOID_STATE_FORWARD;
            break;

        case AVOID_STATE_BACK:
            /* 后退 */
            Car_Backward(60);
            Delay_ms(500);
            Car_TurnLeft(80);
            Delay_ms(300);
            g_avoid_state = AVOID_STATE_FORWARD;
            break;

        case AVOID_STATE_FORWARD:
            /* 前进一段距离后回到IDLE继续检测 */
            Car_Forward(60);
            Delay_ms(500);
            g_avoid_state = AVOID_STATE_IDLE;
            break;

        default:
            g_avoid_state = AVOID_STATE_IDLE;
            break;
    }
}
