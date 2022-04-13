/******************************************************
 * 红外解码头文件
 * by          Gordon
 * 时间        2019/3/19
 * 文件名      infrared.c
 * 原文件名    infrared.c
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#include "common.h"
#include "include.h"
#include "infrared.h"

//IRD  InfraRed Data 红外传来的数据
uint8 Infrared_Data[4];

uint16 get_high_time(void)
{
  uint16 hightime;
  pit_time_start(PIT2);
   while(IRD==1)
  {
      if(pit_time_get_us(PIT2)>10000)
          return 0;
  }
  hightime = pit_time_get_us(PIT2);

  return hightime;
}


uint16 get_low_time(void)
{
  uint16 lowtime;
  pit_time_start(PIT2);
  while(IRD==0)
  {
      if(pit_time_get_us(PIT2)>10000)
          return 0;
  }
  lowtime = pit_time_get_us(PIT2);
  return lowtime;
}

uint8 decode (void)
{
    uint8 i,j,byt=0;
    uint16 time1,time2,time = 0;
    time1 = get_low_time(); //us
    if((time1<8000) || (time1>9500)) return 0;
    time2 = get_high_time();
    if((time2<4000) || (time2>5000))  return 0;

    for(i=0;i<=3;i++)   //数据反码接收在第四位出现问题，故舍去
    {
        for(j=0;j<=7;j++)
        {
            while(IRD==0);
            time = get_high_time();
            if((time>300)&&(time<800))
            {
                byt >>= 1;       //为0
            }
            else if((time>1400)&&(time<1800))
            {
                byt >>= 1;
                byt |= 0x80;
            }
            else
            {
                return 0;
            }
        }
        Infrared_Data[i] = byt;
  }
  systick_delay_ms(10);
  return Infrared_Data[2];

}

