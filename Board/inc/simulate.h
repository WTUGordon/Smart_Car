/******************************************************
 * by          Gordon
 * 时间        2019/4/25
 * 文件名      simualte.c
 * 内容        模拟IIC
 * 软件        IAR8.3
 * 单片机      MK60DN512ZVLQ10

***************************************************/
#ifndef __SIMULATE_H__
#define __SIMULATE_H__




extern void Simulate_Init();
uint8 Simulate_Read (uint8 add, uint8 reg);
void Simluate_Write (uint8 add, uint8 reg, uint8 dat);


#endif