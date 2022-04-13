/******************************************************
 * by          Gordon
 * 时间        2019/4/6
 * 文件名      upper_computerhc
 * 内容        上位机发送数据
 * 软件        IAR8.3
 * 单片机      MK60DN512ZVLQ10

****************************************************/
#ifndef __upper_computer_H__
#define __upper_computer_H__

#include  "common.h"
extern void send_oscilloscope (uint8 *data, uint16 size);
extern void send_camera (uint8 *data, uint16 size);


#endif