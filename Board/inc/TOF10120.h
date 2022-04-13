/******************************************************
 * by          Gordon
 * 时间        2019/4/25
 * 文件名      TOF10120.h
 * 内容        TOF10120激光
 * 软件        IAR8.3
 * 单片机      MK60DN512ZVLQ10

****************************************************/

#ifndef __TOF10120_H__
#define __TOF10120_H__

typedef enum Data_Mode
{
    Real = 1,    //实时距离
    Smooth = 0   //滤波距离
} Data_Mode;


#define  TOF10120_ADD    0xA4  //TOF10120 address

/*IIC Register Address*/
#define  TOF10120_RDL    0x00  // real distense low (only read)
#define  TOF10120_RDH    0x01  // real distense high (only read)
#define  TOF10120_SDL    0x04  // smooth distance low (only read)
#define  TOF10120_SDH    0x05  // smooth distance high (only read)
#define  TOF10120_DEL    0x06  // distance error low (read/write)
#define  TOF10120_DEH    0x07  // distance error high (read/write)
#define  TOF10120_DMC    0x08  // distance mode control (read/write)
#define  TOF10120_DSM    0x09  // distance send mode (read/write)
#define  TOF10120_MDL    0x0c  // max measure distance low (only read)
#define  TOF10120_MDH    0x0d  // max measure distance high (only read)
#define  TOF10120_SAD    0x0f  // iic subordinate address (read/write)


/*function*/
void TOF10120_init (Data_Mode mode); //TOF初始化
uint16 TOF_read (Data_Mode mode);    //距离读取
uint16 TOF_Eread (void);            //距离偏差读取
uint16 TOF_Mread (void);            //最大测量距离读取


#endif