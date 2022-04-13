/******************************************************
 * by          Gordon
 * 时间        2019/4/6
 * 文件名      upper_computer.c
 * 内容        上位机发送数据
 * 软件        IAR8.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#include "include.h"
#include "upper_computer.h"

/*发送8个16位数据到虚拟示波器*/
void send_oscilloscope (uint8 *data ,uint16 size)
{
    uart_putchar (UART4, 0x03);
    uart_putchar (UART4, 0xFC);//帧头

    uart_putbuff (UART4, data, size); //数据

    uart_putchar (UART4, 0xFC);
    uart_putchar (UART4, 0x03);//帧尾

}

/*发送二值化数据到上位机*/
void send_camera (uint8 *data, uint16 size)
{
    uart_putchar (UART4, 0x01);
    uart_putchar (UART4, 0xFE);//帧头

    uart_putbuff (UART4, data, size); //图像

    uart_putchar (UART4, 0xFE);
    uart_putchar (UART4, 0x01);//帧尾

}

