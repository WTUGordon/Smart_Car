/******************************************************
 * 
 * by          Gordon
 * 时间        2019/3/3
 * 文件名      beep.h
 * 原文件名    无
 * 内容        蜂鸣器功能
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#ifndef _beep_H_
#define _beep_H_

extern void beep_init(void);
extern void beep_on(void);
extern void beep_off(void);
extern void time_delay_ms(uint32 ms);

#endif