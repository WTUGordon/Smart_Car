/******************************************************
 * by          Gordon
 * 时间        2019/3/9
 * 文件名      direction.c
 * 内容        电感方向处理
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#include  "include.h"
#include  "math.h"

int16 g_ValueOfAD[5]={0};		//获取的电感值
int16 g_ValueOfADFilter[5]={0};	//阶梯滤波的电感值
float g_fDirectionError[3];		//方向偏差（g_fDirectionError[0]为一对水平电感的差比和偏差.g_fDirectionError[1]为一对垂直电感的差比和偏差）
float g_fDirection_dot[4];
float g_fLastdot=0;
float g_fdot;

float g_fDirectionError_dot;    //偏差变化量
float Last_Errordot = 0;        //上次偏差变化量
int32 E_Gyro=0, E_Gyro_last=0;  //水平角速度

float DirFuzzy_P; //模糊P
float DirFuzzy_D; //模糊D
float Mid_Max;    //中间电感
float Direct_PWM = 0;//水平方向PWM
float Dir_PWM = 0;   //斜方向PWM

uint8 Round_State = 0;   //环岛状态，1正在进环，2在环内，3正在出环，4脱离环岛
uint8 LFlag = 0;         //左环岛标志
uint8 RFlag = 0;         //右环岛标志

uint8 M_Round_Count = 0; //中电感计数
uint8 R_Round_Count = 0; //右电感计数
uint8 L_Round_Count = 0; //左电感计数
float Round_Flag = 3000;  //两边电感标志
float M_Round_Flag = 1000; //中电感标志

int16 Accle_X;
int16 CONT;

/*变形前后方向各标志的变化*/
void Trans_Dir_Flag (void)
{
    if ((Trans_Flag&0x01)) //水平状态
    {
    Round_Flag = 2500;
    M_Round_Flag = 1800;
    }
    else            //直立状态
    {
    Round_Flag = 1500;
    M_Round_Flag = 1000;
//    if (Angle < )
    }
}
/*解算角速度，作为方向D*/
void E_Angle_Read (void)
{
    uint8 buf[6];
    IIC_ReadMultByteFromSlave(FXAS21002_ADDR,0x01,6,buf);
    Pause();
    E_Gyro = (int16_t)((uint16_t)buf[4]<<8 | (uint16_t)buf[5]);
    E_Gyro *= 3.8;  //参数放大
    E_Gyro = (int16)(E_Gyro*0.7+E_Gyro_last*0.3);
    E_Gyro_last=E_Gyro;
}

/*速度分段调节*/
float Dir_Speed (float Spe)
{
    float P = 1.0;
    if (Spe < 1500)
        P = 1.0;
    else if (Spe <= 1700)
        P = 1.0;
    else if (Spe <= 1900)
        P = 1.0;
    else if (Spe <= 2000)
        P = 1.0;

    return P;
}

/*
角度补偿函数
输入：方向偏差
输出：补偿系数
By  ：Gordon
*/
float Ang = 0;
float Angle_Offset (void)
{
    float A=0, B=0, C=0;
    Ang += (Angle - Ang)/10;
    A = -0.009*Ang;
    B = 1.82 * exp (A) + 3.15;
    C = (int16)533 / B;
    C /= 100;
    C -= 0.1;

//    if (Angle > 38)
//    {
//    if (Angle < 72)
//        C = 1.1;
//    else if (Angle < 113)
//        C = 1.2;
//    else if (Angle < 167)
//        C = 1.3;
//    else if (Angle < 256)
//        C = 1.4;
//    else
//        C = 1.5;
//    }

    return C;
}

/**********************************************函数定义*****************************************************
* 函数名字: void DirectionControl(void)
* 输入参数: void
* 输出参数: void
* 功    能: 方向控制函数
* 小    组: BBO
* 日    期: 2019/3/22
************************************************************************************************************/
    float PPWM = 0;
    float DPWM = 0;
    float Offset = 1.0;
void Direction_Control(void)
{
    float DSP=1.0;
//    DSP = Dir_Speed (Speed_Set);
    E_Angle_Read () ;
    Read_ADC();
    if ((Star_Flag&0x04)&&g_ValueOfAD[2]<50 && (Bar_State == 0 || Bar_State == 5))   Stop_Car();  //出赛道保护
	g_ValueOfAD[0] = (g_ValueOfAD[0] < 10? 10:g_ValueOfAD[0]);	//五个电感值限幅
	g_ValueOfAD[1] = (g_ValueOfAD[1] < 10? 10:g_ValueOfAD[1]);
	g_ValueOfAD[2] = (g_ValueOfAD[2] < 10? 10:g_ValueOfAD[2]);
	g_ValueOfAD[3] = (g_ValueOfAD[3] < 10? 10:g_ValueOfAD[3]);
    g_ValueOfAD[4] = (g_ValueOfAD[4] < 10? 10:g_ValueOfAD[4]);

	g_fDirectionError[2] = (float)(g_ValueOfAD[0] - g_ValueOfAD[4])/(g_ValueOfAD[0] + g_ValueOfAD[4] + 2*g_ValueOfAD[2]);//水平电感的差比和作为偏差
	g_fDirectionError[2] = (g_fDirectionError[2]>= 1? 1:g_fDirectionError[2]);	//偏差限幅
	g_fDirectionError[2] = (g_fDirectionError[2]<=-1?-1:g_fDirectionError[2]);

    g_fDirection_dot[3] = g_fDirection_dot[2];
    g_fDirection_dot[2] = g_fDirection_dot[1];
    g_fDirection_dot[1] = g_fDirection_dot[0];
    g_fDirection_dot[0] = g_fDirectionError[2];
    g_fdot = g_fDirection_dot[0]+g_fDirection_dot[1]-g_fDirection_dot[2]-g_fDirection_dot[3];
    g_fLastdot += (g_fdot-g_fLastdot)/5;

	g_fDirectionError[1] = (float)(g_ValueOfAD[1] - g_ValueOfAD[3])/(g_ValueOfAD[1] + g_ValueOfAD[3]);//垂直电感的差比和作为偏差
	g_fDirectionError[1] = (g_fDirectionError[1]>= 1? 1:g_fDirectionError[1]);	//偏差限幅
	g_fDirectionError[1] = (g_fDirectionError[1]<=-1?-1:g_fDirectionError[1]);
//    g_fDirectionError[1] = RANGE_FLOAT(g_fDirectionError[1], -0.5, 0.5);

//    //角度补偿
//    float temp = 0;
//    temp =(int16) 100/(0.94 - 0.0022*Angle);
//    temp /= 100;
//    if (Trans_Flag == 0)
//        g_fDirectionError[0] *= 1.4;
    /*角度补偿*/
    Offset = Angle_Offset();

    DirFuzzy_P = Fuzzy_P(g_fDirectionError[2]*10,g_fLastdot*40);

    DirFuzzy_D = Fuzzy_D(g_fDirectionError[2]*10,g_fLastdot*40);


    PPWM = g_fDirectionError[2]*DirFuzzy_P;
    PPWM *= DSP;    // 速度补偿
    DPWM = E_Gyro*DirFuzzy_D;
    Direct_PWM = PPWM + DPWM;
    /*斜电感*/
    Dir_PWM = g_fDirectionError[1]*8000 + E_Gyro*0;
    Direct_PWM += 0*Dir_PWM;
    Direct_PWM = RANGE_FLOAT(Direct_PWM, -2000, 2000);

    //以下为环岛处理
    if((Star_Flag & 0x80)&&(Run_Flag==1))
    {
        Trans_Dir_Flag();// 获取环岛各标志位
        switch (Round_State)
        {
        case 0: break;                         //无环
        case 1: break;                         //有环
        case 2: Direct_PWM = Dir_PWM;break;    //入环
        case 3: break;                         //环内
        case 4: Direct_PWM = Direct_PWM+Dir_PWM*0;break; //出环
        }
        /*下面是状态任务*/
        if (0 == Round_State){
            /*中间电感进环检测*/
#if 0    //找切点
            if(g_ValueOfAD[2] > M_Round_Flag)
            {
                if (g_ValueOfAD[2] > Mid_Max)
                {
                    M_Round_Count = 0;
                    Mid_Max = g_ValueOfAD[2];
                }
                else
                    M_Round_Count++;
            }
            else
            {
                Mid_Max = 0;
                M_Round_Count = 0;
            }
#else   //固定值
            if(g_ValueOfAD[2] > M_Round_Flag)
                M_Round_Count++;
            else
                M_Round_Count = 0;
#endif
            /*左边电感进环检测*/
            if (g_ValueOfAD[1] > Round_Flag)
                L_Round_Count++;
            else if ((g_ValueOfAD[1]<Round_Flag/2) || (g_ValueOfAD[3]<Round_Flag/2))
                L_Round_Count = 0;
            /*右边电感进环检测*/
            if (g_ValueOfAD[3] > Round_Flag)
                R_Round_Count++;
            else if ((g_ValueOfAD[1]<Round_Flag/2) || (g_ValueOfAD[3]<Round_Flag/2))
                R_Round_Count = 0;
        }
        if (2 == Round_State){
            BarSpeed = FTM_QUAD_get(FTM1) + FTM_QUAD_get(FTM2);
            Pulse_Cont += (BarSpeed - Last_Speed);
            CONT += Pulse_Cont/10;
            Last_Speed = BarSpeed;
            E_Gyro =(int16)Last_Gyro + (E_Gyro-Last_Gyro)/10;
            if(Last_Gyro)
                Accle_X += E_Gyro/100;   //陀螺仪积分得到角度
            Last_Gyro = E_Gyro;
        }
        if (4 == Round_State){
#if 1    //找切点
            if(g_ValueOfAD[2] > M_Round_Flag)
            {
                if (g_ValueOfAD[2] > Mid_Max)
                {
                    M_Round_Count = 0;
                    Mid_Max = g_ValueOfAD[2];
                }
                else
                    M_Round_Count++;
            }
            else
            {
                Mid_Max = 0;
                M_Round_Count = 0;
            }
#else   //固定值
            if(g_ValueOfAD[2] > M_Round_Flag)
                M_Round_Count++;
            else
                M_Round_Count = 0;
#endif
        }
        if (5 == Round_State){
            BarSpeed = FTM_QUAD_get(FTM1) + FTM_QUAD_get(FTM2);
            Pulse_Cont += (BarSpeed - Last_Speed);
            CONT += Pulse_Cont/10;
            Last_Speed = BarSpeed;
        }
        /*下面是状态判别*/
        if ((0==Round_State) && (M_Round_Count>=3) && (L_Round_Count||R_Round_Count)){   //有环岛?！
            beep_on();
            M_Round_Count = 0;
            Round_State=1;
            if (L_Round_Count > R_Round_Count)
                RFlag = 1;
            else if (L_Round_Count < R_Round_Count)
                LFlag = 1;
        }
        if (1==Round_State){                                                         //确认准备
            if ((g_fDirectionError[1]>0&&LFlag) || (g_fDirectionError[1]<0&&RFlag))
                Round_State=2;
            else if (g_ValueOfAD[2] < M_Round_Flag/2)
                Round_State = 0;
        }
        if ((2==Round_State) && ((abs(Accle_X)>3000)&& (CONT>3000))){  //进入环岛，切换
            Round_State=3;
            LFlag = 0;
            RFlag = 0;
            beep_off();
            Accle_X = 0;
            Last_Gyro = 0;
            Pulse_Cont = 0;
            CONT = 0;
            Last_Speed = 0;
        }
        if ((3==Round_State) && (g_ValueOfAD[1]>Round_Flag || g_ValueOfAD[3]>Round_Flag)){ //快要出环啦
            Round_State=4;
            beep_on();
        }
        if ((4==Round_State) && (M_Round_Count>1)){ //第二次路过切点
            Round_State = 5;
            beep_off();
        }
        if ((5==Round_State) && (CONT>6000)){  //远离环岛
            Round_State = 0;
            Accle_x = 0;
            Last_Gyro = 0;
            CONT = 0;
            Last_Speed = 0;
            beep_off();
        }
    }
    Direct_PWM = RANGE_FLOAT(Direct_PWM, -2500, 2500);
}



/**********************************************函数定义*****************************************************
* 函数名字: void Read_ADC(void)
* 输入参数: void
* 输出参数: void
* 功    能: 读取并处理ADC值
* 小    组: BBO
* 日    期: 2019/3/9
************************************************************************************************************/
void Read_ADC(void)
{
     int16  i,j,k,temp;
     int16  ad_valu[5][5],ad_valu1[5],ad_sum[5];
     int16  ValueOfADOld[5],ValueOfADNew[5];

     for(i=0;i<5;i++)
     {
         ad_valu[0][i]=ADC_Ave(AD1, ADC_12bit, 5);  		// AD1自左向右1
         ad_valu[1][i]=ADC_Ave(AD2, ADC_12bit, 5);     		// AD2自左向右2
         ad_valu[2][i]=ADC_Ave(AD3, ADC_12bit, 5);  		// AD3自左向右3
         ad_valu[3][i]=ADC_Ave(AD4, ADC_12bit, 5);     		// AD4自左向右4
         ad_valu[4][i]=ADC_Ave(AD5, ADC_12bit, 5);     		// AD4自左向右5
     }

/*=========================冒泡排序升序==========================*///舍弃最大值和最小值
     for(i=0;i<5;i++)
     {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
              if(ad_valu[i][k] > ad_valu[i][k+1])        //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              }
           }
        }
     }
/*===========================中值滤波=================================*/
     for(i=0;i<5;i++)    //求中间三项的和
     {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valu1[i] = ad_sum[i] / 3;
     }

	for(i=0;i<5;i++)            //将数值中个位数除掉
	 {
	 	g_ValueOfAD[i] = (int16)(ad_valu1[i]/10*10);

		//采集梯度平滑，每次采集最大变化40
		ValueOfADOld[i] = g_ValueOfADFilter[i];
		ValueOfADNew[i] = g_ValueOfAD[i];

		if(ValueOfADNew[i]>=ValueOfADOld[i])
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>50?(ValueOfADOld[i]+50):ValueOfADNew[i]);
		else
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-60?(ValueOfADOld[i]-60):ValueOfADNew[i]);
	 }
}