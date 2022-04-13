/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK66_i2c.h
 * @brief      i2c����ͷ�ļ�
 * @author     ɽ��Ƽ�
 * @version    v6.0
 * @date       2013-07-12
 * @note       Ŀǰ��ʵ��������д�Ĵ������ܣ��������ܴ�ʵ��
 */

#ifndef     __MK66_I2C_H__
#define     __MK66_I2C_H__

#include "common.h"
#include "MK66_gpio.h"

#define IIC_SCL_PIN  PTE1 //ģ��IIC��SCL�ź�  1.�޸����ż����޸�IIC�ӿ�
#define IIC_SDA_PIN  PTE0 //ģ��IIC��SDA�ź�

#define SDA_I()  GPIO_DDR(IIC_SDA_PIN, 0);	//����
#define SDA_O() GPIO_DDR(IIC_SDA_PIN, 1);	//���

#define IIC_SCL    PTXn_T(IIC_SCL_PIN,OUT)  //SCL            2.�޸����ż����޸�IIC�ӿ�
#define IIC_SDA    PTXn_T(IIC_SDA_PIN,OUT)  //SDA
#define READ_SDA   PTXn_T(IIC_SDA_PIN,IN)   //����SDA


/*---------------------------------------------------------------
            IIC�ڲ�����
----------------------------------------------------------------*/
void IIC_Begin(void);			        //����IIC��ʼ�ź�
void IIC_Cease(void);	  	            //����IICֹͣ�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				    //IIC������ACK�ź�
uint8_t IIC_WaitAck(void); 		        //IIC�ȴ�ACK�ź�
void IIC_SendByte(uint8_t data);        //IIC����һ���ֽ�
uint8_t IIC_ReadByte(uint8_t ack);       //IIC��ȡһ���ֽ�


/*---------------------------------------------------------------
            IIC�û�����
----------------------------------------------------------------*/
void IIC_Init(void);                    //��ʼ��IIC��IO��
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
extern void Pause(void);


#endif