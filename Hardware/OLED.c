/*
 * OLED.c - OLED显示屏驱动模块
 * 
 * 本模块驱动0.96寸OLED显示屏（128×64像素）
 * 使用软件模拟I2C协议与OLED通信
 * 
 * 硬件连接：
 *   SCL = PB8（I2C时钟线）
 *   SDA = PB9（I2C数据线）
 *   OLED地址 = 0x78（写操作）
 * 
 * 显示原理：
 *   OLED内部有128×64个像素点
 *   分为8页（Page0~Page7），每页8行像素
 *   每页128列，每列对应一个字节的1位
 *   所以写入一个字节可以控制8个像素点
 * 
 * 字体：
 *   大字体（8×16）：每个字符占8列×16行=2页
 *   小字体（4×8）：每个字符占4列×8行=1页
 */

#include "stm32f10x.h"
#include "OLED_Font.h"

/* 引脚配置（软件模拟I2C） */
#define OLED_W_SCL(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))  // SCL时钟线
#define OLED_W_SDA(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))  // SDA数据线

/*============================================================================*/
/*                              I2C底层通信函数                                */
/*============================================================================*/

/*
 * OLED_I2C_Init - 初始化I2C引脚
 * 将PB8(SCL)和PB9(SDA)配置为开漏输出
 * 开漏输出可以配合外部上拉电阻实现双向通信
 */
void OLED_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;    // 开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	OLED_W_SCL(1);  // SCL初始高电平
	OLED_W_SDA(1);  // SDA初始高电平
}

/*
 * OLED_I2C_Start - I2C起始信号
 * 在SCL高电平时，SDA从高电平跳变到低电平
 * 表示一次I2C通信的开始
 */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

/*
 * OLED_I2C_Stop - I2C停止信号
 * 在SCL高电平时，SDA从低电平跳变到高电平
 * 表示一次I2C通信的结束
 */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/*
 * OLED_I2C_SendByte - I2C发送一个字节
 * 高位先发，每个bit在SCL低电平时设置数据，高电平时采样
 * 发送完8位后，额外产生一个时钟用于从机应答
 * 本驱动不检查应答信号
 * 
 * Byte: 要发送的字节
 */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(!!(Byte & (0x80 >> i)));  // 从高位开始逐位发送
		OLED_W_SCL(1);   // SCL高电平，从机采样数据
		OLED_W_SCL(0);   // SCL低电平，准备下一位
	}
	OLED_W_SCL(1);	// 额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
}

/*============================================================================*/
/*                              OLED命令/数据写入                              */
/*============================================================================*/

/*
 * OLED_WriteCommand - 向OLED写入命令
 * 命令字节用于配置OLED的工作模式
 * 比如设置显示位置、亮度等
 * 
 * Command: 命令字节
 */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		// 从机地址（写操作）
	OLED_I2C_SendByte(0x00);		// 控制字节：0x00表示后面是命令
	OLED_I2C_SendByte(Command);     // 发送命令
	OLED_I2C_Stop();
}

/*
 * OLED_WriteData - 向OLED写入数据
 * 数据字节用于设置像素点的亮灭
 * 1=点亮，0=熄灭
 * 
 * Data: 数据字节
 */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		// 从机地址（写操作）
	OLED_I2C_SendByte(0x40);		// 控制字节：0x40表示后面是数据
	OLED_I2C_SendByte(Data);        // 发送数据
	OLED_I2C_Stop();
}

/*============================================================================*/
/*                              光标设置与清屏                                 */
/*============================================================================*/

/*
 * OLED_SetCursor - 设置光标位置
 * OLED的显示缓冲区分为8页（Page0~Page7）
 * 每页对应8行像素
 * 
 * Y: 页地址（0~7），对应行范围：Y*8 ~ Y*8+7
 * X: 列地址（0~127），对应列范围：X ~ X+7
 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					// 设置Y位置（页地址）
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	// 设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			// 设置X位置低4位
}

/*
 * OLED_Clear - 清屏
 * 将所有像素点设置为熄灭状态
 * 遍历8页×128列，每列写入0x00
 */
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)          // 遍历8页
	{
		OLED_SetCursor(j, 0);         // 设置页地址
		for(i = 0; i < 128; i++)      // 遍历128列
		{
			OLED_WriteData(0x00);     // 写入0x00（全部熄灭）
		}
	}
}

/*============================================================================*/
/*                              大字体显示函数（8×16像素）                    */
/*============================================================================*/

/*
 * OLED_ShowChar - 显示一个字符（大字体）
 * 每个字符从字库中取出16个字节
 * 前8个字节显示上半部分（8×8），后8个字节显示下半部分（8×8）
 * 
 * Line: 行位置（1~4）
 * Column: 列位置（1~16）
 * Char: 要显示的ASCII字符
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		// 设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);			// 显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	// 设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		// 显示下半部分内容
	}
}

/*
 * OLED_ShowString - 显示字符串（大字体）
 * 逐个字符调用OLED_ShowChar显示
 * 遇到'\0'结束
 * 
 * Line: 起始行位置（1~4）
 * Column: 起始列位置（1~16）
 * String: 要显示的字符串
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/*
 * OLED_Pow - 次方函数
 * 用于计算10的N次方、16的N次方等
 * 在显示数字时用于提取每一位
 * 
 * X: 底数
 * Y: 指数
 * 返回值：X的Y次方
 */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/*
 * OLED_ShowNum - 显示十进制数（大字体）
 * 将数字的每一位提取出来，逐个显示
 * 比如Number=123, Length=3 → 显示"123"
 * 
 * Line: 起始行位置（1~4）
 * Column: 起始列位置（1~16）
 * Number: 要显示的数字（0~4294967295）
 * Length: 数字的位数（1~10）
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*
 * OLED_ShowSignedNum - 显示带符号十进制数（大字体）
 * 正数显示"+"，负数显示"-"
 * 
 * Line: 起始行位置（1~4）
 * Column: 起始列位置（1~16）
 * Number: 要显示的数字（-2147483648~2147483647）
 * Length: 数字的位数（1~10）
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*
 * OLED_ShowHexNum - 显示十六进制数（大字体）
 * 使用0~9和A~F表示
 * 
 * Line: 起始行位置（1~4）
 * Column: 起始列位置（1~16）
 * Number: 要显示的数字（0~0xFFFFFFFF）
 * Length: 数字的位数（1~8）
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/*
 * OLED_ShowBinNum - 显示二进制数（大字体）
 * 只显示0和1
 * 
 * Line: 起始行位置（1~4）
 * Column: 起始列位置（1~16）
 * Number: 要显示的数字（0~65535）
 * Length: 数字的位数（1~16）
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/*============================================================================*/
/*                              OLED初始化                                     */
/*============================================================================*/

/*
 * OLED_Init - OLED初始化
 * 
 * 初始化序列（按照SSD1306数据手册）：
 *   1. 上电延时
 *   2. 初始化I2C引脚
 *   3. 关闭显示
 *   4. 设置显示时钟分频
 *   5. 设置多路复用率
 *   6. 设置显示偏移
 *   7. 设置显示开始行
 *   8. 设置左右方向
 *   9. 设置上下方向
 *   10. 设置COM引脚配置
 *   11. 设置对比度
 *   12. 设置预充电周期
 *   13. 设置VCOMH电压
 *   14. 设置整个显示打开
 *   15. 设置正常显示
 *   16. 使能充电泵
 *   17. 开启显示
 *   18. 清屏
 */
void OLED_Init(void)
{
	uint32_t i, j;
	
	for (i = 0; i < 1000; i++)			// 上电延时
	{
		for (j = 0; j < 1000; j++);
	}
	
	OLED_I2C_Init();			// 端口初始化
	
	OLED_WriteCommand(0xAE);	// 关闭显示
	
	OLED_WriteCommand(0xD5);	// 设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	// 设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	// 设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	// 设置显示开始行
	
	OLED_WriteCommand(0xA1);	// 设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	// 设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	// 设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	// 设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	// 设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	// 设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	// 设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	// 设置正常/倒转显示

	OLED_WriteCommand(0x8D);	// 设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	// 开启显示
		
	OLED_Clear();				// OLED清屏
}

/*============================================================================*/
/*                          小字体显示函数（4×8像素）                         */
/*============================================================================*/

/*
 * OLED_ShowSmallChar - 显示一个小字符（4×8像素）
 * 每个字符从字库中取出4个字节
 * 显示在1页×4列的区域
 * 
 * Line: 行位置（1~8）
 * Column: 列位置（1~32）
 * Char: 要显示的ASCII字符
 */
void OLED_ShowSmallChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1), (Column - 1) * 4);		// 设置光标位置
	for (i = 0; i < 4; i++)
	{
		OLED_WriteData(OLED_F4x8[Char - ' '][i]);		// 显示字符内容
	}
}

/*
 * OLED_ShowSmallString - 显示小字符串（4×8像素）
 */
void OLED_ShowSmallString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowSmallChar(Line, Column + i, String[i]);
	}
}

/*
 * OLED_ShowSmallNum - 显示小十进制数（4×8像素）
 */
void OLED_ShowSmallNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowSmallChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*
 * OLED_ShowSmallSignedNum - 显示小带符号十进制数（4×8像素）
 */
void OLED_ShowSmallSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowSmallChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowSmallChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowSmallChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*
 * OLED_ShowSmallHexNum - 显示小十六进制数（4×8像素）
 */
void OLED_ShowSmallHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowSmallChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowSmallChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/*
 * OLED_ShowSmallBinNum - 显示小二进制数（4×8像素）
 */
void OLED_ShowSmallBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowSmallChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/*============================================================================*/
/*                              小字体显示示例                                 */
/*============================================================================*/

/*
 * OLED_TestSmallFont - 小字体显示测试函数
 * 用于测试小字体显示功能
 * 显示各种状态信息
 */
void OLED_TestSmallFont(void)
{
	OLED_Clear();
	
	// 显示标题
	OLED_ShowSmallString(1, 1, "SmartCar V1.0");
	
	// 显示状态信息
	OLED_ShowSmallString(2, 1, "L:60% FWD R:60% FWD");
	OLED_ShowSmallString(3, 1, "Speed:60% Dir:FWD");
	OLED_ShowSmallString(4, 1, "Time:01:30 Batt:3.8V");
	OLED_ShowSmallString(5, 1, "IR:●●●○ US:25cm");
	OLED_ShowSmallString(6, 1, "BT:Connected RSSI:-65");
	OLED_ShowSmallString(7, 1, "Mode:Auto Error:0");
	OLED_ShowSmallString(8, 1, "Debug:Running Test");
}

/*============================================================================*/
/*                              显示能力对比                                   */
/*============================================================================*/

// 大字体（8×16像素）：
// - 显示行数：4行（1~4）
// - 显示列数：16列（1~16）
// - 总字符数：64个
// - 适合显示少量重要信息

// 小字体（4×8像素）：
// - 显示行数：8行（1~8）
// - 显示列数：32列（1~32）
// - 总字符数：256个
// - 适合显示详细信息

// 使用建议：
// 1. 重要标题使用大字体（OLED_ShowString）
// 2. 详细信息使用小字体（OLED_ShowSmallString）
// 3. 可以混合使用两种字体
