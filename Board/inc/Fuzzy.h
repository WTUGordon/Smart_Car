/******************************************************
 * by          Gordon
 * 时间        2019/3/31
 * 文件名      Fuzzy.h
 * 内容        模糊控制函数
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#ifndef __Fuzzy_H__
#define __Fuzzy_H__

#include  "common.h"

extern float Fuzzy_P (float P,float D);
extern float Fuzzy_D (float P,float D);
extern float Fuzzy_I (float P,float D);

#endif