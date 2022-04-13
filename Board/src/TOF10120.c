/******************************************************
 * by          Gordon
 * 时间        2019/4/25
 * 文件名      TOF10120.c
 * 内容        TOF10120激光IIC处理(模拟IIC)
 * 软件        IAR8.3
 * 单片机      MK60DN512ZVLQ10

****************************************************/

#include "include.h"
#include "TOF10120.h"


/*!
 *  @brief      TOF初始化)
 *  @param      Data_Mode   数据方式(Real、Smooth)
 *  @return                 void
 */
void TOF10120_init (Data_Mode mode)
{
    Simulate_Init(); //模拟IIC初始化

    Simluate_Write(TOF10120_ADD, TOF10120_DSM, 1);
    Pause();
    Simluate_Write(TOF10120_ADD, TOF10120_DMC, mode);
    Pause();
}
/*数据读取*/
uint16 TOF_read (Data_Mode mode)
{
    uint16 Data=0x0000;
    uint8  D[2] = {0};
    if (mode == Real)
    {
        D[0] = Simulate_Read (TOF10120_ADD, TOF10120_RDL);
        Pause();
        D[1] = Simulate_Read (TOF10120_ADD, TOF10120_RDH);
        Pause();
    }
    if (mode == Smooth)
    {
        D[0] = Simulate_Read (TOF10120_ADD, TOF10120_SDL);
        Pause();
        D[1] = Simulate_Read (TOF10120_ADD, TOF10120_SDH);
        Pause();
    }

    Data = ((Data | D[0])<<8) | D[1];
    DELAY_US(30);
    return Data;
}

/*距离偏差*/
uint16 TOF_Eread (void)
{
    uint16 Data = 0x0000;
    uint8  D[2];

    D[0] = Simulate_Read(TOF10120_ADD, TOF10120_DEL);
    Pause();
    D[1] = Simulate_Read(TOF10120_ADD, TOF10120_DEH);
    Pause();

    Data = ((Data | D[0])<<8) | D[1];
    DELAY_US(30);
    return Data;
}

/*最大距离*/
uint16 TOF_Mread (void)
{
    uint16 Data = 0x0000;
    uint8  D[2];

    D[0] = Simulate_Read(TOF10120_ADD, TOF10120_MDL);
    Pause();
    D[1] = Simulate_Read(TOF10120_ADD, TOF10120_MDH);
    Pause();

    Data = ((Data | D[0])<<8) | D[1];
    DELAY_US(30);
    return Data;
}



