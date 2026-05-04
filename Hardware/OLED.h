#ifndef __OLED_H
#define __OLED_H_

#include "stm32f10x.h"

/*
 * OLED显示屏驱动（0.96寸，128×64像素，I2C接口）
 * 
 * 本驱动支持两种字体：
 * 
 * 1. 大字体（8×16像素）：
 *    每个字符占8列×16行像素
 *    显示区域：4行（1~4）× 16列（1~16）
 *    适合显示标题、重要信息
 * 
 * 2. 小字体（4×8像素）：
 *    每个字符占4列×8行像素
 *    显示区域：8行（1~8）× 32列（1~32）
 *    适合显示详细信息、调试数据
 * 
 * I2C引脚（软件模拟I2C）：
 *   SCL = PB8（时钟线）
 *   SDA = PB9（数据线）
 * 
 * OLED地址：0x78（7位地址0x3C左移1位）
 */

/*============================================================================*/
/*                              大字体函数（8×16像素）                        */
/*============================================================================*/

void OLED_Init(void);                                                       // 初始化OLED
void OLED_Clear(void);                                                      // 清屏

// 大字体显示函数（8×16像素，4行×16列）
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);                // 显示一个字符（行：1~4，列：1~16）
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);           // 显示字符串（行：1~4，列：1~16）
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);      // 显示十进制数（行：1~4，列：1~16）
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length); // 显示带符号十进制数
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);   // 显示十六进制数
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);   // 显示二进制数

/*============================================================================*/
/*                              小字体函数（4×8像素）                         */
/*============================================================================*/

// 小字体显示函数（4×8像素，8行×32列）
void OLED_ShowSmallChar(uint8_t Line, uint8_t Column, char Char);                // 显示一个小字符（行：1~8，列：1~32）
void OLED_ShowSmallString(uint8_t Line, uint8_t Column, char *String);           // 显示小字符串（行：1~8，列：1~32）
void OLED_ShowSmallNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);      // 显示小十进制数
void OLED_ShowSmallSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length); // 显示小带符号数
void OLED_ShowSmallHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);   // 显示小十六进制数
void OLED_ShowSmallBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);   // 显示小二进制数

#endif
