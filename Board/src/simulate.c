/******************************************************
 * by          Gordon
 * 时间        2019/4/25
 * 文件名      simualte.c
 * 内容        模拟IIC
 * 软件        IAR8.3
 * 单片机      MK60DN512ZVLQ10

****************************************************/

#include "include.h"
#include "simulate.h"

#define SDA             gpio_get (Simulate_SDA)
#define SDA0()          gpio_set (Simulate_SDA, 0)		//IO口输出低电平
#define SDA1()          gpio_set (Simulate_SDA, 1)		//IO口输出高电平
#define SCL0()          gpio_set (Simulate_SCL, 0)		//IO口输出低电平
#define SCL1()          gpio_set (Simulate_SCL, 1)		//IO口输出高电平
#define DIR_OUT()      gpio_ddr (Simulate_SDA, GPO)      //输出方向
#define DIR_IN()        gpio_ddr (Simulate_SDA, GPI)      //输入方向

#define ack 1      //主应答
#define no_ack 0   //从应答

/*延迟*/
void Simiic_delay (void)
{
    uint16 i = 200;
    while(i--);
}
/*IIC开始*/
void IIC_Start (void)
{
    SDA1();;
    SCL1();;
    Simiic_delay();
    SDA0();;
    Simiic_delay();
    SCL0();;
}
/*IIC停止*/
void IIC_Stop (void)
{
    SDA0();;
    SCL0();;
    Simiic_delay();
    SCL1();;
    Simiic_delay();
    SDA1();;
    Simiic_delay();
}
/*ACK应答*/
void IIC_SendACK(uint8 ack_dat)
{
    SCL0();
    Simiic_delay();
    if(ack_dat)   SDA0();
    else         SDA1();
    SCL1();
    Simiic_delay();
    SCL0();
    Simiic_delay();
}
/*等待ACK*/
void Wait_Ack (void)
{
    uint8 error = 0;
    SCL0();
    DIR_IN();
    Simiic_delay();

    SCL1();
    Simiic_delay();

    if(SDA)  //异常！
    {
        DIR_OUT();
        SCL0();
        error = 1;
    }
    DIR_OUT();
    SCL0();
    Simiic_delay();
    if (error)  IIC_Stop();
}
/*发送一字节*/
void Send_Byte (uint8 dat)
{
    uint8 i = 8;
    SCL0();
    Simiic_delay();

    while(i--)
    {
        if(dat & 0x80)  SDA1();
        else            SDA0();
        dat <<= 1;
        SCL1();
        Simiic_delay();
        SCL0();
        Simiic_delay();
    }
    Wait_Ack();
}
/*读取一字节*/
uint8 Read_Byte (void)
{
    uint8 dat = 0x00;
    SCL0();
    Simiic_delay();
    SDA1();
    DIR_IN();
    for(uint8 i=0; i<8; i++)
    {
        Simiic_delay();
        SCL0();
        Simiic_delay();
        SCL1();
        Simiic_delay();
        dat <<= 1;
        if(SDA)  dat += 1;
    }
    DIR_OUT();
    SCL0();
    Simiic_delay();
    IIC_SendACK(no_ack);
    return dat;
}
/*地址写入*/
void Simluate_Write (uint8 add, uint8 reg, uint8 dat)
{
    IIC_Start();
    Send_Byte(add | 0x00);
    Send_Byte(reg);
    DELAY_US(50);
    Send_Byte(dat);
    IIC_Stop();
}
/*地址读取*/
uint8 Simulate_Read (uint8 add, uint8 reg)
{
    uint8 dat;
    IIC_Start();
    Send_Byte(add | 0x00);
    Send_Byte(reg);
    IIC_Stop();
    DELAY_US(50);

    IIC_Start();
    Send_Byte(add | 0x01);
    dat = Read_Byte();
    IIC_Stop();
    DELAY_US(30);
    return dat;
}
/*初始化*/
void Simulate_Init(void)
{
    gpio_init (Simulate_SCL, GPO,1);
	gpio_init (Simulate_SDA, GPO,1);

	port_init_NoALT (Simulate_SCL, ODO | PULLUP);//ODO
	port_init_NoALT (Simulate_SDA, ODO | PULLUP);
}

