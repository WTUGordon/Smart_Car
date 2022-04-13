/******************************************************
* by          FQF
* 时间        2019/3/6
* 文件名      control.c
* 内容        控制小车角度和速度
* 软件        IAR8.3
* 单片机      MK66FX
******************************************************/
#include "include.h"
#include "control.h"

PID Angle_PID,
      Speed_PID,
      Horizon_PID ;	//定义角度，角速度，速度控制的PID参数结构体

int32 Angle_Tar= ANGLE_SET, //目标角度
          Speed_Set=0,                 //目标速度   //  * 0.173 / 0.06 = X*2.8/2 =  期望速度  mm/s
          Last_Speed_Set=0,
          Speed_Out=0;                //速度增量

int32 New_Angle=ANGLE_SET,
          Old_Angle=ANGLE_SET;

int32 Speed[5]={0};   //速度

uint8 Run_Flag=0;//运行标志
//float Speed_Set = 1000;		//  * 0.173 / 0.06 = X*2.88/2 =  期望速度  mm/s

                               //88.5,   0,  -5.3           最后一项为积分限幅
                              //160,   0,  -6.82
float Angle_Vel[4] = {70,   0,  -3,  500};   //太原工业
                    //7,0,-3.59
//增量式PID系数   如果速度调节过大 那么将I减小 如果速度调节过小 将I增大  P是乘的是变化率  I乘的偏差
float Speed_Vel[4] = {0.45,//0.42,//0.7,
                                    0.04,//0.03,//0.03,
                                    0.0,//
                                    500};

float Horizon_Vel[4]={0.3,
                                    0.001,
                                    0.0,
                                    50};

int32 Balance_Control(int32 Angle_Zero)
{
    int32 Balance_PWM=0;
    Angle_Read();

    Balance_PWM=(int32)Angle_Control(&Angle_PID,(int32)Angle,(int32)Gyro_z ,Angle_Tar);

    if (Trans_Flag & 0x01&&Slope_State==0) //水平状态
        Balance_PWM=RANGE_INT32(Balance_PWM,0,4500);
    else
        Balance_PWM=RANGE_INT32(Balance_PWM,-4500,4500);

//    if(Slope_State==1)
//    {
//            Balance_PWM=RANGE_INT32(Balance_PWM,0,4500);
//    }
    if(Speed_Flag)
    {
        Speed_Flag=0;
        Speed[0]=Speed_Get();

        if(Speed[2]>1000&&Speed[0]>1000)
            Run_Flag=1;
        if(Run_Flag==1&&Speed[2]<10&&Speed[0]<10&&(Star_Flag&0x02))
        {
            Run_Flag=0;
            Stop=1;
        }

        Speed[3]=(Speed[0]+Speed[1]+Speed[2])/3;

        Speed[3]=(Speed[3]>(Speed[4]+50)?(Speed[4]+50):Speed[3]);
        Speed[3]=(Speed[3]<(Speed[4]-200)?(Speed[4]-200):Speed[3]);

        Speed[4]=Speed[3];

        Speed[1]=Speed[0];
        Speed[2]=Speed[1];



        if (Trans_Flag & 0x01)//水平状态
        {
            Speed_Out  =  (int32)PID_Realize(&Horizon_PID,Speed[3],Speed_Set);  //注意正负号
        }
        else
        {
            Speed_Out  = (int32)Speed_Control(&Speed_PID,Speed[0],Speed_Set);
        }

        Angle_Tar=Angle_Zero+Speed_Out;


        if (!(Trans_Flag & 0x01)) //直立状态
            Angle_Tar=RANGE_INT32(Angle_Tar,210,350);
        else
            Angle_Tar=RANGE_INT32(Angle_Tar,-10,100);

    }
    return Balance_PWM;
}

float Angle_Control(PID *sptr,int32 Angle,int32 Gyro, int32 Angle_Point)
{
		//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError;	// 当前误差
	float Realize;	// 最后得出的实际增量
	iError = Angle_Point - Angle;	// 计算当前误差
	Realize = sptr->KP * iError
			+ sptr->KD * Gyro;
	return Realize;	// 返回实际值
}




float Last_Increase=0;
float Speed_Control(PID *sptr, int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError=0;	//当前误差
	float Value=0;
    float Increase=0;	//最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差


    if(iError > 700)
    {
        Increase= -90;
    }
    else if(iError > -500&&iError <= 700)
    {
        Value =  sptr->KP * (iError - sptr->LastError)
                  + sptr->KI * iError
                  + sptr->KD * (iError - 2 * sptr->LastError + sptr->PrevError);
        Value=RANGE_FLOAT(Value,-30,20);
        Increase=Last_Increase - Value;
    }
    else
    {
        Increase=-20;
    }


	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据

    //Increase=RANGE_INT32(Increase,-90,45);

    Last_Increase= Increase;
	return Increase;	// 返回增量
}


 float	 Last_Realize=0;	// 上次输出值
// 位置式PID控制
float PID_Realize(PID *sptr,  int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError;	// 当前误差
	float	 Realize;	// 最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差

	sptr->SumError += sptr->KI * iError;	// 误差积分
	if (sptr->SumError >= sptr->KT)
	{
		sptr->SumError = sptr->KT;
	}
	else if (sptr->SumError <= -sptr->KT)
	{
		sptr->SumError = -sptr->KT;
	}

    if(iError<=-200)
    {
        Realize=-50;
    }
    else
    {
        Realize = sptr->KP * iError
                + sptr->SumError
                + sptr->KD * (iError - sptr->LastError);
    }
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据

    Last_Realize=Realize;
	return Realize;	// 返回实际值
}

int32 Angle_SpeedOut(int32 NewAngle, int32 LastAngle, float PeriodCount)
{
	int32  Anglefav;
	int32  AngleOut;
	Anglefav = NewAngle - LastAngle;
	AngleOut = (int32)(Anglefav /PeriodCount+0.5)  + LastAngle;

	return AngleOut;

}

void PID_Parameter_Init(PID *sptr, float *pidset)
{
    sptr->KP=pidset[0];
    sptr->KI=pidset[1] ;
    sptr->KD=pidset[2];
    sptr->KT=pidset[3];
	sptr->SumError  = 0;
    sptr->PrevError = 0;	//Error[-2]
	sptr->LastError = 0;	//Error[-1]
	sptr->LastData = 0;
}

