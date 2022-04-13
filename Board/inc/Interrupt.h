/******************************************************
 * 龙邱库移植到野火库。
 * by          Gordon
 * 时间        2019/3/6
 * 文件名      Interrupt.h
 * 内容        存放工程的所有中断服务函数
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/
#ifndef __Interrupt_H__
#define __Interrupt_H__

extern int8 Up_State;
extern int L_FTM_Count;             //左电机脉冲计数
extern int R_FTM_Count;             //右电机脉冲计数
extern int16 Send_data[8];
extern uint8 Star_Flag;
extern uint8 Bar_State;
extern int32 Pulse_Cont;
extern int16 Ang_Barrier;
extern float Bar_P;
extern float Bar_D;
extern int16 Angle_Er;
extern int16 BarSpeed;
extern int32 Last_Speed;
extern int32 Cont;
extern uint8 Break_Flag;
extern int32 Pulse_Cont;
extern uint8 Trans_Flag;
extern int16 Last_Gyro;
extern uint8 Speed_Flag;
extern uint8 Break_Count;

extern int32 PWM;
extern uint16 Barrier_Distance;
extern uint8  symbol ;
extern int32 Accle_x;
extern void PID_Barrier(int16 Aim_Angle);



void Dispose_Barrier(void);     //路障处理
void Data_Barrier (void);       //路障数值
void PID_Barrier(int16 Aim_Angle);
extern void PIT0_IRQHandler(void);    //PIT0中断服务函数
extern void PIT1_IRQHandler(void);    //PIT1中断服务函数
extern void PORTA_IRQHandler(void);   //端口A中断服务函数
extern void PORTD_IRQHandler(void);   //端口E中断服务函数
extern void PORTE_IRQHandler(void);
extern void DMA0_IRQHandler(void);    //DMA中断服务函数


#endif