#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */

#include  "MK66_wdog.h"
#include  "MK66_gpio.h"     //IO口操作
#include  "MK66_uart.h"     //串口
#include  "MK66_SysTick.h"
#include  "MK66_i2c.h"      //I2C
#include  "MK66_spi.h"      //SPI
#include  "MK66_ftm.h"      //FTM
#include  "MK66_pit.h"      //PIT
#include  "MK66_rtc.h"      //RTC
#include  "MK66_adc.h"      //ADC
#include  "MK66_dma.h"      //DMA
#include  "MK66_mcg.h"     //mcg
#include  "MK66_port.h"

/*下面是工程涉及到的外设模块驱动库文件*/

#include  "LED.H"                 //LED
#include  "KEY.H"                 //KEY
#include  "camera.h"              //摄像头总头文件
#include  "OLED.h"                //OLED头文件
#include  "8700_2100.h"           //九轴接收数据
#include  "direction.h"
#include  "beep.h"                //蜂鸣器

/*下面是BBO自己写的工程所需的头文件*/

#include  "Interrupt.h"           //所有的中断服务函数在这里
#include  "Car_Init.h"            //小车全部的初始化
#include  "infrared.h"            //红外接收
#include  "angle.h"               //角度接收处理
#include  "speed.h"               //速度处理&输出
#include  "direction.h"           //方向控制
#include  "Fuzzy.h"               //模糊控制
#include  "control.h"             //小车控制
#include  "simulate.h"         //模拟IIC
#include  "TOF10120.h"      //激光相关
#include  "upper_computer.h"      //山外上位机协议

//#include  "VCAN_computer.h"     //多功能调试助手



#endif  //__INCLUDE_H__
