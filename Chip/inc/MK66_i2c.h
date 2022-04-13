/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK66_i2c.h
 * @brief      i2c驱动头文件
 * @author     山外科技
 * @version    v6.0
 * @date       2013-07-12
 * @note       目前仅实现主机读写寄存器功能，其他功能待实现
 */

#ifndef     __MK66_I2C_H__
#define     __MK66_I2C_H__

#include "common.h"
#include "MK66_gpio.h"

#define IIC_SCL_PIN  PTE1 //模拟IIC的SCL信号  1.修改引脚即可修改IIC接口
#define IIC_SDA_PIN  PTE0 //模拟IIC的SDA信号

#define SDA_I()  GPIO_DDR(IIC_SDA_PIN, 0);	//输入
#define SDA_O() GPIO_DDR(IIC_SDA_PIN, 1);	//输出

#define IIC_SCL    PTXn_T(IIC_SCL_PIN,OUT)  //SCL            2.修改引脚即可修改IIC接口
#define IIC_SDA    PTXn_T(IIC_SDA_PIN,OUT)  //SDA
#define READ_SDA   PTXn_T(IIC_SDA_PIN,IN)   //输入SDA


/*---------------------------------------------------------------
            IIC内部函数
----------------------------------------------------------------*/
void IIC_Begin(void);			        //发送IIC开始信号
void IIC_Cease(void);	  	            //发送IIC停止信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				    //IIC不发送ACK信号
uint8_t IIC_WaitAck(void); 		        //IIC等待ACK信号
void IIC_SendByte(uint8_t data);        //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);       //IIC读取一个字节


/*---------------------------------------------------------------
            IIC用户函数
----------------------------------------------------------------*/
void IIC_Init(void);                    //初始化IIC的IO口
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
extern void Pause(void);


#endif