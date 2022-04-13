#ifndef __DIRECTION_H__
#define __DIRECTION_H__

#include  "common.h"

extern float Direct_PWM;
extern float Dir_PWM;
extern uint8 L_Flag_Round;
extern uint8 R_Flag_Round;
extern uint8 Round_State;
extern uint8 L_Out_Round;
extern uint8 Flag_Round;
extern float PPWM;
extern float DPWM;
extern float Ang;
extern float Offset;
extern uint8 L_Round_Count;
extern uint8 R_Round_Count;
extern float g_fLastdot;
extern int16 g_ValueOfAD[5];
extern float g_fDirectionError[3];
extern float g_fDirectionError_dot;
extern float Last_Errordot;
extern int16 Accle_X;
extern int16 CONT;
extern float DirFuzzy_P;
extern float DirFuzzy_D;
extern int32 E_Gyro;
void Read_ADC(void);
void E_Angle_Read (void);
void Direction_Control(void) ;
#endif