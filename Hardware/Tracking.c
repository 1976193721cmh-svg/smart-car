/*
 * Tracking.c - 循迹模块
 * 
 * 本模块负责实现小车的自动循迹功能
 * 使用4个TCRT5000红外传感器检测地面黑线
 * 
 * 循迹原理：
 *   黑线对红外光吸收强，白底反射强
 *   传感器检测到黑线时输出高电平，白底时输出低电平
 * 
 * 传感器布局（从左到右）：
 *   左2(PB14)  左1(PB12)  右1(PB13)  右2(PB15)
 * 
 * 循迹策略：
 *   通过比较左右传感器的状态，判断小车偏离黑线的方向和程度
 *   然后控制左右轮速度差，使小车回到黑线上
 */

#include "Tracking.h"
#include "Motor.h"
#include "../System/Delay.h"

/*
 * Tracking_Init - 初始化循迹传感器
 * 将PB12~PB15配置为浮空输入
 * 这些引脚连接到TCRT5000传感器的DO（数字输出）引脚
 */
void Tracking_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;     // 浮空输入模式
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = TRACK_SENSOR_L2_PIN | TRACK_SENSOR_L1_PIN |
                    TRACK_SENSOR_R1_PIN | TRACK_SENSOR_R2_PIN;
    GPIO_Init(GPIOB, &gpio);
}

/*
 * Tracking_Read - 读取4个循迹传感器状态
 * 
 * 返回值（bit3~bit0）：
 *   bit3 = 左2（最左边传感器）
 *   bit2 = 左1
 *   bit1 = 右1
 *   bit0 = 右2（最右边传感器）
 * 
 * 每个位：1=检测到黑线，0=检测到白底
 */
uint8_t Tracking_Read(void)
{
    uint8_t val = 0;
    if (GPIO_ReadInputDataBit(TRACK_SENSOR_L2_PORT, TRACK_SENSOR_L2_PIN))
        val |= TRACK_L2_MASK;   // 左2检测到黑线
    if (GPIO_ReadInputDataBit(TRACK_SENSOR_L1_PORT, TRACK_SENSOR_L1_PIN))
        val |= TRACK_L1_MASK;   // 左1检测到黑线
    if (GPIO_ReadInputDataBit(TRACK_SENSOR_R1_PORT, TRACK_SENSOR_R1_PIN))
        val |= TRACK_R1_MASK;   // 右1检测到黑线
    if (GPIO_ReadInputDataBit(TRACK_SENSOR_R2_PORT, TRACK_SENSOR_R2_PIN))
        val |= TRACK_R2_MASK;   // 右2检测到黑线
    return val;
}

/*
 * Tracking_Run - 循迹运行函数
 * 
 * 在main循环中周期调用，根据传感器状态控制电机
 * 
 * 循迹逻辑详解：
 * 
 *   理想状态：黑线在左1和右1之间
 *   此时左1或右1检测到黑线，小车直行
 * 
 *   情况1：左2检测到黑线（严重偏左）
 *     → 小车向左偏太多了，需要大幅度右转
 *     → 先原地左转，直到右1检测到黑线
 *     → 然后稍微右回一点，让线回到中间
 * 
 *   情况2：右2检测到黑线（严重偏右）
 *     → 小车向右偏太多了，需要大幅度左转
 *     → 先原地右转，直到左1检测到黑线
 *     → 然后稍微左回一点，让线回到中间
 * 
 *   情况3：只有左1检测到黑线（轻微偏左）
 *     → 前进中右转微调
 * 
 *   情况4：只有右1检测到黑线（轻微偏右）
 *     → 前进中左转微调
 * 
 *   情况5：左1和右1都检测到黑线
 *     → 黑线在正中间，直行
 * 
 *   情况6：全白（没有检测到任何黑线）
 *     → 可能线断了或者出界了，直行尝试找回
 * 
 *   情况7：全黑（4个传感器都检测到黑线）
 *     → 到达终点（黑线尽头），停车
 */
void Tracking_Run(void)
{
    uint8_t sensors = Tracking_Read();

    /* ---- 严重偏左：左2检测到黑线 ---- */
    if (sensors & TRACK_L2_MASK) {
        // 向左转，直到右1检测到黑线
        Car_TurnLeft(TRACK_SPEED_TURN);
        
        while (1) {
            sensors = Tracking_Read();
            if (sensors & TRACK_R1_MASK) break;  // 右1碰到线了
            if (sensors & TRACK_R2_MASK) break;  // 右2也碰到了（过冲了）
            Car_TurnLeft(TRACK_SPEED_TURN);
            
        }
        // 右1已检测到，稍微右回一点，让线在左1和右1之间
        Car_ForwardTurnRight(TRACK_SPEED_SLOW);
        
        return;
    }

    /* ---- 严重偏右：右2检测到黑线 ---- */
    if (sensors & TRACK_R2_MASK) {
        // 向右转，直到左1检测到黑线
        Car_TurnRight(TRACK_SPEED_TURN);
        
        while (1) {
            sensors = Tracking_Read();
            if (sensors & TRACK_L1_MASK) break;  // 左1碰到线了
            if (sensors & TRACK_L2_MASK) break;  // 左2也碰到了（过冲了）
            Car_TurnRight(TRACK_SPEED_TURN);
           
        }
        // 左1已检测到，稍微左回一点，让线在左1和右1之间
        Car_ForwardTurnLeft(TRACK_SPEED_SLOW);
        
        return;
    }

    /* ---- 轻微偏左：只有左1检测到黑线 ---- */
    if ((sensors & TRACK_L1_MASK) && !(sensors & TRACK_R1_MASK)) {
        Car_ForwardTurnRight(TRACK_SPEED_SLOW);  // 前进中右转微调
        return;
    }

    /* ---- 轻微偏右：只有右1检测到黑线 ---- */
    if (!(sensors & TRACK_L1_MASK) && (sensors & TRACK_R1_MASK)) {
        Car_ForwardTurnLeft(TRACK_SPEED_SLOW);   // 前进中左转微调
        return;
    }

    /* ---- 中间：左1+右1都检测到，或全白 ---- */
    if ((sensors & TRACK_L1_MASK) && (sensors & TRACK_R1_MASK)) {
        Car_Forward(TRACK_SPEED_DEFAULT);        // 直行
        return;
    }

    /* ---- 全白 ---- */
    if (sensors == 0x00) {
        Car_Forward(TRACK_SPEED_DEFAULT);        // 直行尝试找回
        return;
    }

    /* ---- 全黑：到达终点 ---- */
    if (sensors == 0x0F) {
        Car_Stop();                              // 停车
        return;
    }

    /* ---- 其他情况：直行 ---- */
    Car_Forward(TRACK_SPEED_DEFAULT);
}
