/******************************************************
 * by          Gordon
 * 时间        2019/3/21
 * 文件名      speed.h
 * 原文件名    speed.h
 * 内容        编码器处理
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#ifndef  __speed_H__
#define  __speed_H__

#define abs(x)   (x>0?x:-x)

void MotorSpeedOut(int32 AngPWM, float DirPWM);
float SpeedPWMOut(float NewspeedPWM ,float LastspeedPWM,uint8_t PeriodCount);
int32 Speed_Get (void);
void Trans_Speed_Control (float g_fExpectSpeed);
float Loca_Speed_Control (float g_fExpectSpeed);
extern void Stop_Car (void);
extern void Slope (void);
extern void Dispose_Slope (void);
extern float Angle_Speed (void);


extern int16 Last_TGyro;
extern int16 TAngle;
extern int16 TGyro_z;
extern int16 AimAngle;
extern float APWM;
extern int16 TAccle_z;
extern uint8 Slope_Flag;
extern int16 Gyro_Last;
extern float TPWM;

extern float Speed_Filter;
extern uint8 Stop;
extern int32 g_fSpeedFilter;
extern int16 L_Count;
extern int16 R_Count;
extern float g_fSpeedError[3];
extern float True_Speed;
extern uint8 Slope_State;
extern int32 PWM_L, PWM_R;

#endif