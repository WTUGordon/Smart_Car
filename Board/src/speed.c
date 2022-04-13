/******************************************************
* by          Gordon
* 时间        2019/3/21
* 文件名      speed.c
* 原文件名    speed.c
* 内容        编码器处理
* 软件        IAR7.3
* 单片机      MK60DN512ZVLQ10

******************************************************/
#include "include.h"
#include "speed.h"

int32 PWM_L=0; //左电机输出
int32 PWM_R=0; //右电机输出
float true_speed  = 200/(1160*0.06*2);//  （Count*一圈周长）/（一圈脉冲数*周期*两个编码器） 单位 mm/s
                              //周长20cm 一圈1160脉冲*2
/*
功能：坡道检测
By  ：Gordon
*/
uint8 Slope_State = 0;
uint8 Slope_Flag = 0;
void Slope (void)
{
    if (Trans_Flag & 0x01) //水平状态
    {
//        if ((g_ValueOfAD[2] > 1350) && (g_ValueOfAD[0] < 1450) && (g_ValueOfAD[4] < 1450) && (Round_State == 0) && (Slope_State == 0))
        if((Speed[3]<700)&& (Slope_State == 0))
        {
            beep_on();
            DELAY_MS(500);
            beep_off();
            Slope_State = 1;
        }
    }
}
void Stop_Car(void);
uint8 Stop = 0; //为1停车
/*
功能 ： 速度控制
输入 ： 角度PWM，方向PWM
输出 ： 真实速度
*/

void MotorSpeedOut(int32 AngPWM, float DirPWM)
{
//	Speed_L = (int32)(AngPWM + DirPWM*0.5);
//	PWM_R = (int32)(AngPWM - DirPWM*1.5);

    /*转速差加少减多*/
    if (DirPWM > 0)
    {
        PWM_L = (int32)(AngPWM + DirPWM*0.5);
        PWM_R = (int32)(AngPWM - DirPWM*1.5);
    }
    else
    {
        PWM_L = (int32)(AngPWM + DirPWM*1.5);
        PWM_R = (int32)(AngPWM - DirPWM*0.5);
    }
	/*转换为占空比*/
	PWM_R /= 50;
	PWM_L /= 50;

	/*限幅，最大90%*/
    PWM_R=RANGE_INT32(PWM_R,-90,90);
    PWM_L=RANGE_INT32(PWM_L,-90,90);

    if (Stop)
        Stop_Car();
    else
    {
        if (PWM_R > 0)
        {
            FTM_PWM_Duty(FTM0, FTM_CH0, 0);
            FTM_PWM_Duty(FTM0, FTM_CH1, PWM_R);
        }
        else
        {
            FTM_PWM_Duty(FTM0, FTM_CH0, -PWM_R);
            FTM_PWM_Duty(FTM0, FTM_CH1, 0);
        }

        if (PWM_L > 0)
        {
            FTM_PWM_Duty(FTM0, FTM_CH2, PWM_L);
            FTM_PWM_Duty(FTM0, FTM_CH3, 0);
        }
        else
        {
            FTM_PWM_Duty(FTM0, FTM_CH2, 0);
            FTM_PWM_Duty(FTM0, FTM_CH3, -PWM_L);
        }
    }
}

int32 L_Count_Last,R_Count_Last;       //电机脉冲计数
int32 R_Acc,L_Acc;     //加速度
uint8 Left_Crazy=0;    // 左电机疯转
uint8 Right_Crazy=0;  // 右电机疯转

/*
功能 ： 计算速度
输入 ： 无
输出 ： 真实速度
*/
int32 Speed_Get (void)
{
    int32 Speed_Now=0;//输出速度
    int32 L_Count,R_Count; //电机脉冲计数
	static int32 Speed_Last = 0;
	static int32 Crazy_Count = 0;
    int32 SpeedSET= Speed_Set/2;

	/******* 右电机速度相关控制 ********/
    R_Count = FTM_AB_Get(FTM2);	// 获取FTM 正交解码 的脉冲数

	R_Acc = R_Count - R_Count_Last;	// 计算加速度

	if (R_Acc > 100)
	{
		Right_Crazy = 1;	// 疯转
	}

	if (Right_Crazy)
	{
		if (R_Acc <= 100)
		{
			if ((R_Count < SpeedSET + 200) && R_Count > 0)
			{
				Right_Crazy = 0;
			}
		}
	}

	if (!Right_Crazy)
	{
		R_Count =(int32)(R_Count*0.9 + R_Count_Last*0.1);
		R_Count_Last = R_Count;	// 更新右轮速度
	}
	else
	{
		R_Count = (int32)(R_Count*0.5 + R_Count_Last*0.5);
		R_Count_Last = R_Count;	// 更新右轮速度
	}
	/******* 右电机速度相关控制结束 ********/

	/******* 左电机速度相关控制 ********/
	L_Count = FTM_AB_Get(FTM1);	// 获取FTM 正交解码 的脉冲数

	L_Acc = L_Count - L_Count_Last;	// 计算加速度
	if (L_Acc > 100)
	{
		Left_Crazy = 1;
	}

	if (Left_Crazy)
	{
		if (L_Acc <= 100)
		{
			if ((L_Count < SpeedSET + 200) && L_Count > 0)
			{
				Left_Crazy = 0;
			}
		}
	}

	if (!Left_Crazy)
	{
		L_Count = (int32)(0.9*L_Count + 0.1*L_Count_Last);	// 低通滤波
		L_Count_Last = L_Count;	// 更新左轮速度
	}
	else
	{
		L_Count = (int32)(0.5*L_Count + 0.5*L_Count_Last);	// 低通滤波
		L_Count_Last = L_Count;	// 更新左轮速度
	}



	/******* 左电机速度相关控制结束 ********/


	if ((Left_Crazy && Right_Crazy) || (Left_Crazy && R_Count < 20) || (Right_Crazy && L_Count < 20)) //疯转停车
	{
		Crazy_Count++;
		if (Crazy_Count >= 40)
		{
			Crazy_Count = 0;
			//Stop = 1;
		}
	}
	else
	{
		Right_Crazy = 0;
	}

	/******* 电机疯转特殊处理 ********/
	if ((Left_Crazy > 0) && (Right_Crazy > 0))
	{
		Speed_Now = Speed_Last;			// 两边都疯转，使用上次速度作为当前实际速度
	}
	else if (Left_Crazy)
	{
		if (R_Count > SpeedSET)
		{
			Speed_Now = Speed_Last;
		}
		else
		{
			Speed_Now = R_Count*2;	// 左电机疯转，使用上次速度作为当前实际速度
		}
	}
	else if (Right_Crazy)
	{
		if (L_Count > SpeedSET)
		{
			Speed_Now = Speed_Last;
		}
		else
		{
			Speed_Now = L_Count*2;	// 右电机疯转，使用上次速度作为当前实际速度
		}
	}
	else
	{
		Speed_Now = (L_Count + R_Count);	// 左右取平均计算车子实际速度
	}

	Speed_Now = (int32)(Speed_Now *0.9 + Speed_Last * 0.1);
	Speed_Last = Speed_Now;

    return Speed_Now;
}

/*
功能 ： 停车
输入 ： 无
输出 ： 无
*/
void Stop_Car(void)
{
	FTM_PWM_Duty(FTM0, FTM_CH0, 0);
	FTM_PWM_Duty(FTM0, FTM_CH1, 0);
	FTM_PWM_Duty(FTM0, FTM_CH2, 0);
	FTM_PWM_Duty(FTM0, FTM_CH3, 0);
	beep_on();
    DELAY_MS(500);
    beep_off();
    while (1);
}

#define SPEEDPERIODFAV  (100)
float SpeedPWMOut(float NewspeedPWM ,float LastspeedPWM,uint8_t PeriodCount)
{
    float  speedPWMfav ;
    float  SpeedOUT ;
    speedPWMfav = NewspeedPWM - LastspeedPWM ;
    SpeedOUT = speedPWMfav *(PeriodCount)/SPEEDPERIODFAV + LastspeedPWM ;

    return SpeedOUT ;

}