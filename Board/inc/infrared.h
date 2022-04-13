/******************************************************
 * 红外解码头文件
 * by          Gordon
 * 时间        2019/3/19
 * 文件名      infrared.h
 * 原文件名    infrared.h
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#ifndef __infrared_H__
#define __infrared_H__

extern uint8 decode (void);
extern void delayus(uint16 us);
extern uint8 Infrared_Data[4];
uint16 get_high_time(void);
uint16 get_low_time(void);

#endif