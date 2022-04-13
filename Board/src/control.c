/******************************************************
* by          FQF
* ʱ��        2019/3/6
* �ļ���      control.c
* ����        ����С���ǶȺ��ٶ�
* ���        IAR8.3
* ��Ƭ��      MK66FX
******************************************************/
#include "include.h"
#include "control.h"

PID Angle_PID,
      Speed_PID,
      Horizon_PID ;	//����Ƕȣ����ٶȣ��ٶȿ��Ƶ�PID�����ṹ��

int32 Angle_Tar= ANGLE_SET, //Ŀ��Ƕ�
          Speed_Set=0,                 //Ŀ���ٶ�   //  * 0.173 / 0.06 = X*2.8/2 =  �����ٶ�  mm/s
          Last_Speed_Set=0,
          Speed_Out=0;                //�ٶ�����

int32 New_Angle=ANGLE_SET,
          Old_Angle=ANGLE_SET;

int32 Speed[5]={0};   //�ٶ�

uint8 Run_Flag=0;//���б�־
//float Speed_Set = 1000;		//  * 0.173 / 0.06 = X*2.88/2 =  �����ٶ�  mm/s

                               //88.5,   0,  -5.3           ���һ��Ϊ�����޷�
                              //160,   0,  -6.82
float Angle_Vel[4] = {70,   0,  -3,  500};   //̫ԭ��ҵ
                    //7,0,-3.59
//����ʽPIDϵ��   ����ٶȵ��ڹ��� ��ô��I��С ����ٶȵ��ڹ�С ��I����  P�ǳ˵��Ǳ仯��  I�˵�ƫ��
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

    if (Trans_Flag & 0x01&&Slope_State==0) //ˮƽ״̬
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



        if (Trans_Flag & 0x01)//ˮƽ״̬
        {
            Speed_Out  =  (int32)PID_Realize(&Horizon_PID,Speed[3],Speed_Set);  //ע��������
        }
        else
        {
            Speed_Out  = (int32)Speed_Control(&Speed_PID,Speed[0],Speed_Set);
        }

        Angle_Tar=Angle_Zero+Speed_Out;


        if (!(Trans_Flag & 0x01)) //ֱ��״̬
            Angle_Tar=RANGE_INT32(Angle_Tar,210,350);
        else
            Angle_Tar=RANGE_INT32(Angle_Tar,-10,100);

    }
    return Balance_PWM;
}

float Angle_Control(PID *sptr,int32 Angle,int32 Gyro, int32 Angle_Point)
{
		//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError;	// ��ǰ���
	float Realize;	// ���ó���ʵ������
	iError = Angle_Point - Angle;	// ���㵱ǰ���
	Realize = sptr->KP * iError
			+ sptr->KD * Gyro;
	return Realize;	// ����ʵ��ֵ
}




float Last_Increase=0;
float Speed_Control(PID *sptr, int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError=0;	//��ǰ���
	float Value=0;
    float Increase=0;	//���ó���ʵ������

	iError = Point - NowData;	// ���㵱ǰ���


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


	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����

    //Increase=RANGE_INT32(Increase,-90,45);

    Last_Increase= Increase;
	return Increase;	// ��������
}


 float	 Last_Realize=0;	// �ϴ����ֵ
// λ��ʽPID����
float PID_Realize(PID *sptr,  int32 NowData, int32 Point)
{
	//��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
	int32 iError;	// ��ǰ���
	float	 Realize;	// ���ó���ʵ������

	iError = Point - NowData;	// ���㵱ǰ���

	sptr->SumError += sptr->KI * iError;	// ������
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
	sptr->PrevError = sptr->LastError;	// ����ǰ�����
	sptr->LastError = iError;		  	// �����ϴ����
	sptr->LastData  = NowData;			// �����ϴ�����

    Last_Realize=Realize;
	return Realize;	// ����ʵ��ֵ
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

