#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */

#include  "MK66_wdog.h"
#include  "MK66_gpio.h"     //IO�ڲ���
#include  "MK66_uart.h"     //����
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

/*�����ǹ����漰��������ģ���������ļ�*/

#include  "LED.H"                 //LED
#include  "KEY.H"                 //KEY
#include  "camera.h"              //����ͷ��ͷ�ļ�
#include  "OLED.h"                //OLEDͷ�ļ�
#include  "8700_2100.h"           //�����������
#include  "direction.h"
#include  "beep.h"                //������

/*������BBO�Լ�д�Ĺ��������ͷ�ļ�*/

#include  "Interrupt.h"           //���е��жϷ�����������
#include  "Car_Init.h"            //С��ȫ���ĳ�ʼ��
#include  "infrared.h"            //�������
#include  "angle.h"               //�ǶȽ��մ���
#include  "speed.h"               //�ٶȴ���&���
#include  "direction.h"           //�������
#include  "Fuzzy.h"               //ģ������
#include  "control.h"             //С������
#include  "simulate.h"         //ģ��IIC
#include  "TOF10120.h"      //�������
#include  "upper_computer.h"      //ɽ����λ��Э��

//#include  "VCAN_computer.h"     //�๦�ܵ�������



#endif  //__INCLUDE_H__
