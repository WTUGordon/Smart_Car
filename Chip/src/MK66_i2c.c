/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK66_i2c.c
 * @brief      i2c��������
 * @author     ɽ��Ƽ�
 * @version    v6.0
 * @date       2013-07-12
 */

#include "common.h"
#include "MK66_port.h"
#include "MK66_i2c.h"
#include "include.h"

/*!
 *  @brief      I2Cͨ�Ž�������Ҫ���õĺ�������
 *  @since      v5.0
 *  @note       ���ͨ��ʧ�ܣ��ɳ����������ʱֵ��ȷ���Ƿ���ʱ���µ�
 */
void Pause(void)
{
    volatile uint16 n = 80;     //ע�⣬�������̫С���ᵼ�¶�ȡ����

    while(n--)
    {
        asm("nop");
    }
}

/******************************************************************************
*��  ����void delay_us(void)
*�����ܣ�IIC��ʱ
*��  ������
*����ֵ����
*��  ע: ��ֲʱֻ��Ҫ��delay_us()�����Լ�����ʱ����
*******************************************************************************/
void delay_us(uint8_t us)
{
    for(int i = 0; i < 100; i++)
    {
        asm("NOP");//core bus 200M  ����´��IIC���� 400K
    }
}

/******************************************************************************
*��  ����void IIC_Init(void)
*�����ܣ�IIC��ʼ��
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void IIC_Init(void)
{
    gpio_init(IIC_SCL_PIN,GPO,1);
    gpio_init(IIC_SDA_PIN,GPO,1);

	IIC_SCL=1;
	IIC_SDA=1;
}
/******************************************************************************
*��  ����void IIC_Start(void)
*�����ܣ�����IIC��ʼ�ź�
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void IIC_Begin(void)
{
	SDA_O(); //sda�����
	IIC_SDA=1;
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0; //START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCL=0; //ǯסI2C���ߣ�׼�����ͻ��������
}

/******************************************************************************
*��  ����void IIC_Stop(void)
*�����ܣ�����IICֹͣ�ź�
*��  ������
*����ֵ����
*��  ע����
*******************************************************************************/
void IIC_Cease(void)
{
	SDA_O(); //sda�����
	IIC_SCL=0;
	IIC_SDA=0; //STOP:when CLK is high DATA change form low to high
    delay_us(4);
	IIC_SCL=1;
	IIC_SDA=1; //����I2C���߽����ź�
    delay_us(4);
}

/******************************************************************************
*��  ��: uint8_t IIC_WaitAck(void)
*������: �ȴ�Ӧ���źŵ��� ����ЧӦ�𣺴ӻ���9�� SCL=0 ʱ SDA ���ӻ�����,
���� SCL = 1ʱ SDA��ȻΪ�ͣ�
*��  ������
*����ֵ��1������Ӧ��ʧ��
0������Ӧ��ɹ�
*��  ע���ӻ���������Ӧ��
*******************************************************************************/
uint8_t IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	SDA_I(); //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ��
	IIC_SDA=1;delay_us(1);
	IIC_SCL=1;delay_us(1);;
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Cease();
			return 1;
		}
	}
	IIC_SCL=0; //ʱ�����0
	return 0;
}

/******************************************************************************
*��  ��: void IIC_Ack(void)
*������: ����ACKӦ�� ������������һ���ֽ����ݺ�����������ACK֪ͨ�ӻ�һ��
�ֽ���������ȷ���գ�
*��  ������
*����ֵ����
*��  ע���������ӻ���Ӧ��
*******************************************************************************/

void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_O();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}

/******************************************************************************
*��  ��: void IIC_NAck(void)
*������: ����NACKӦ�� ���������������һ���ֽ����ݺ�����������NACK֪ͨ�ӻ�
���ͽ������ͷ�SDA,�Ա���������ֹͣ�źţ�
*��  ������
*����ֵ����
*��  ע���������ӻ���Ӧ��
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_O();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/******************************************************************************
*��  ����void IIC_SendByte(uint8_t txd)
*��  �ܣ�IIC����һ���ֽ�
*��  ����data Ҫд������
*����ֵ����
*��  ע���������ӻ���
*******************************************************************************/
void IIC_SendByte(uint8_t data)
{
    uint8_t t;
    SDA_O();
    IIC_SCL=0; //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
        IIC_SDA=(data&0x80)>>7;
        data<<=1;
        delay_us(1);
        IIC_SCL=1;
        delay_us(1);
        IIC_SCL=0;
        delay_us(1);
    }
}

/******************************************************************************
*��  ����uint8_t IIC_ReadByte(uint8_t ack)
*��  �ܣ�IIC��ȡһ���ֽ�
*��  ����ack=1 ʱ���������ݻ�û������ ack=0 ʱ����������ȫ���������
*����ֵ����
*��  ע���ӻ���������
*******************************************************************************/
uint8_t IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	SDA_I(); //SDA����Ϊ����ģʽ �ȴ����մӻ���������
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0;
        delay_us(1);
        IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++; //�ӻ����͵ĵ�ƽ
        delay_us(1);
    }
    if(ack)
        IIC_Ack(); //����ACK
    else
        IIC_NAck(); //����nACK
    return receive;
}

/******************************************************************************
*��  ����uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
*�����ܣ���ȡָ���豸 ָ���Ĵ�����һ��ֵ
*��  ����I2C_Addr  Ŀ���豸��ַ
reg	     �Ĵ�����ַ
*buf      ��ȡ����Ҫ�洢�ĵ�ַ
*����ֵ������ 1ʧ�� 0�ɹ�
*��  ע����
*******************************************************************************/
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	IIC_Begin();
	IIC_SendByte(I2C_Addr);	 //���ʹӻ���ַ
	if(IIC_WaitAck()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		IIC_Cease();
		return 1;
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
	IIC_WaitAck();

	IIC_Begin();
	IIC_SendByte(I2C_Addr+1); //�������ģʽ
	IIC_WaitAck();
	*buf=IIC_ReadByte(0);
    IIC_Cease(); //����һ��ֹͣ����
	return 0;
}

/******************************************************************************
*��  ����uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr��uint8_t buf))
*�����ܣ�д��ָ���豸 ָ���Ĵ�����һ��ֵ
*��  ����I2C_Addr  Ŀ���豸��ַ
reg	     �Ĵ�����ַ
buf       Ҫд�������
*����ֵ��1 ʧ�� 0�ɹ�
*��  ע����
*******************************************************************************/
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	IIC_Begin();
	IIC_SendByte(I2C_Addr); //���ʹӻ���ַ
	if(IIC_WaitAck())
	{
		IIC_Cease();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
    IIC_WaitAck();
	IIC_SendByte(data);
	if(IIC_WaitAck())
	{
		IIC_Cease();
		return 1; //����д��ʧ��
	}
	IIC_Cease(); //����һ��ֹͣ����

    //return 1; //status == 0;
	return 0;
}

/******************************************************************************
*��  ����uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
*�����ܣ���ȡָ���豸 ָ���Ĵ����� length��ֵ
*��  ����dev     Ŀ���豸��ַ
reg	   �Ĵ�����ַ
length  Ҫ�����ֽ���
*data   ���������ݽ�Ҫ��ŵ�ָ��
*����ֵ��1�ɹ� 0ʧ��
*��  ע����
*******************************************************************************/
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    uint8_t count = 0;
	uint8_t temp;
	IIC_Begin();
	IIC_SendByte(dev); //���ʹӻ���ַ
	if(IIC_WaitAck())
	{
		IIC_Cease();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
    IIC_WaitAck();
	IIC_Begin();
	IIC_SendByte(dev+1); //�������ģʽ
	IIC_WaitAck();
    for(count=0;count<length;count++)
	{
		if(count!=(length-1))
            temp = IIC_ReadByte(1); //��ACK�Ķ�����
		else
            temp = IIC_ReadByte(0); //���һ���ֽ�NACK

		data[count] = temp;
	}
    IIC_Cease(); //����һ��ֹͣ����
    //return count;
    return 0;
}

/******************************************************************************
*��  ����uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
*�����ܣ�������ֽ�д��ָ���豸 ָ���Ĵ���
*��  ����dev     Ŀ���豸��ַ
reg	   �Ĵ�����ַ
length  Ҫд���ֽ���
*data   Ҫд������ݽ�Ҫ��ŵ�ָ��
*����ֵ��1�ɹ� 0ʧ��
*��  ע����
*******************************************************************************/
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{

 	uint8_t count = 0;
	IIC_Begin();
	IIC_SendByte(dev); //���ʹӻ���ַ
	if(IIC_WaitAck())
	{
		IIC_Cease();
		return 1; //�ӻ���ַд��ʧ��
	}
	IIC_SendByte(reg); //���ͼĴ�����ַ
    IIC_WaitAck();
	for(count=0;count<length;count++)
	{
		IIC_SendByte(data[count]);
		if(IIC_WaitAck()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
		{
			IIC_Cease();
			return 1; //����д��ʧ��
		}
	}
	IIC_Cease(); //����һ��ֹͣ����

	return 0;
}
