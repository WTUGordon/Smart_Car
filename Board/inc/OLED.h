/******************************************************
 * 龙邱库移植到野火库。
 * by          Gordon
 * 时间        2019/1/11
 * 文件名      OLED.h
 * 原文件名    LQ12864.h
 * 内容        OLED库
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#ifndef __OLED_H__
#define __OLED_H__

extern void LCD_Init(void);
extern void LCD_CLS(void);
extern void LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void LCD_Print(unsigned char x, unsigned char y, unsigned char ch[]);
extern void LCD_PutPixel(unsigned char x,unsigned char y);
extern void LCD_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char gif);
extern void Draw_LibLogo(void);
extern void Draw_BMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char * bmp);
extern void Draw_Road(void);
extern void LCD_ClrDot(unsigned char x);  //线性CCD使用
extern void dis_bmp(uint16 high, uint16 width, uint8 *p,uint8 value);//OV7725使用

#endif