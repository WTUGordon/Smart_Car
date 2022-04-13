/******************************************************
 * 龙邱库移植到野火库。
 * by          Gordon
 * 时间        2019/3/3
 * 文件名      Karman.h
 * 原文件名    Karman.h
 * 内容        卡尔曼对九轴的滤波
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#ifndef __Karman_H__
#define __Karman_H__ 


extern int16 Angle;



void Kalman_Filter(int Gyro,int Accel);










#endif