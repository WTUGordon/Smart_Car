/******************************************************
 * by          FQF
 * ʱ��        2019/4/17
 * �ļ���      control.h
 * ����        ƽ�⴮�����ƣ�����butter-fly��
 * ���        IAR8.3
 * ��Ƭ��      MK60DN512ZVLQ10

******************************************************/

#ifndef __BALANCE_H__
#define __BALANCE_H__

#define ANGLE_SET   300//300
#define HORIZON_SET  40


typedef struct PID
{
    float KP;
    float KI;
    float KD;
    float KT;
	float SumError;	//����ۼ�
    int32 PrevError;	//Error[-2]
	int32 LastError;	//Error[-1]
	int32 LastData;	//Speed[-1]
}PID;


extern uint8 Run_Flag;//���б�־
extern int32  Speed_Out;
extern int32 Speed_Set;
extern int32 Last_Speed_Set;
extern int32 Speed[5];
extern int32 Angle_Zero;
extern int32 L_Gyro;
extern int32 Angle_Tar;
extern PID Angle_PID, Speed_PID, Horizon_PID;

extern float Horizon_Vel[4]  ,Angle_Vel[4], Speed_Vel[4] ;

void PID_Parameter_Init(PID *sptr, float *pidset) ;

float Angle_Control(PID *sptr,int32 Angle,int32 Gyro, int32 Angle_Point);
float Speed_Control(PID *sptr, int32 NowData, int32 Point);
float PID_Realize(PID *sptr,  int32 NowData, int32 Point);
int32 Angle_SpeedOut(int32 NewAngle, int32 LastAngle, float PeriodCount);
int32 Balance_Control(int32 Angle_Zero);


#endif