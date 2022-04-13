/******************************************************
 * 龙邱库移植到野火库。
 * by          Gordon
 * 时间        2019/3/21
 * 文件名      angle.h
 * 原文件名    angle.h
 * 内容        九轴数据处理
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#ifndef __angle_H__
#define __angle_H__

typedef struct{
  float X;
  float Y;
  float Z;
  float Xoffset;
  float Yoffset;
  float Zoffset;
  uint8 GyrooffsetOK;
}Gyrotypedef;


typedef struct{
  float X;
  float Y;
  float Z;
}Acctypedef;

extern  Acctypedef  ACC_Real;
extern Gyrotypedef GYRO_Real;

extern double g_fGravityAngle ;
extern int16 Gyro_z;
extern double g_fCarAngle ;
extern float Angle;
extern float Yaw;
int16 AAangPWMOut(int16 NewAangPWM ,int16 LastAangPWM,uint8_t PeriodCount);
void Angle_Read (void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);//g表陀螺仪，a表加计
void Matrix_KalmanFilter(float Gyro,float Accel);
void KalmanFilter(float ACC_Angle);
 void  Filter(float ACC_Angle);
 float Kaman_ACC(float signal) ;
#endif