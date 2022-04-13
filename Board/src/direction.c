/******************************************************
 * by          Gordon
 * ʱ��        2019/3/9
 * �ļ���      direction.c
 * ����        ��з�����
 * ���        IAR7.3
 * ��Ƭ��      MK60DN512ZVLQ10

******************************************************/

#include  "include.h"
#include  "math.h"

int16 g_ValueOfAD[5]={0};		//��ȡ�ĵ��ֵ
int16 g_ValueOfADFilter[5]={0};	//�����˲��ĵ��ֵ
float g_fDirectionError[3];		//����ƫ�g_fDirectionError[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ��.g_fDirectionError[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ�
float g_fDirection_dot[4];
float g_fLastdot=0;
float g_fdot;

float g_fDirectionError_dot;    //ƫ��仯��
float Last_Errordot = 0;        //�ϴ�ƫ��仯��
int32 E_Gyro=0, E_Gyro_last=0;  //ˮƽ���ٶ�

float DirFuzzy_P; //ģ��P
float DirFuzzy_D; //ģ��D
float Mid_Max;    //�м���
float Direct_PWM = 0;//ˮƽ����PWM
float Dir_PWM = 0;   //б����PWM

uint8 Round_State = 0;   //����״̬��1���ڽ�����2�ڻ��ڣ�3���ڳ�����4���뻷��
uint8 LFlag = 0;         //�󻷵���־
uint8 RFlag = 0;         //�һ�����־

uint8 M_Round_Count = 0; //�е�м���
uint8 R_Round_Count = 0; //�ҵ�м���
uint8 L_Round_Count = 0; //���м���
float Round_Flag = 3000;  //���ߵ�б�־
float M_Round_Flag = 1000; //�е�б�־

int16 Accle_X;
int16 CONT;

/*����ǰ�������־�ı仯*/
void Trans_Dir_Flag (void)
{
    if ((Trans_Flag&0x01)) //ˮƽ״̬
    {
    Round_Flag = 2500;
    M_Round_Flag = 1800;
    }
    else            //ֱ��״̬
    {
    Round_Flag = 1500;
    M_Round_Flag = 1000;
//    if (Angle < )
    }
}
/*������ٶȣ���Ϊ����D*/
void E_Angle_Read (void)
{
    uint8 buf[6];
    IIC_ReadMultByteFromSlave(FXAS21002_ADDR,0x01,6,buf);
    Pause();
    E_Gyro = (int16_t)((uint16_t)buf[4]<<8 | (uint16_t)buf[5]);
    E_Gyro *= 3.8;  //�����Ŵ�
    E_Gyro = (int16)(E_Gyro*0.7+E_Gyro_last*0.3);
    E_Gyro_last=E_Gyro;
}

/*�ٶȷֶε���*/
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
�ǶȲ�������
���룺����ƫ��
���������ϵ��
By  ��Gordon
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

/**********************************************��������*****************************************************
* ��������: void DirectionControl(void)
* �������: void
* �������: void
* ��    ��: ������ƺ���
* С    ��: BBO
* ��    ��: 2019/3/22
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
    if ((Star_Flag&0x04)&&g_ValueOfAD[2]<50 && (Bar_State == 0 || Bar_State == 5))   Stop_Car();  //����������
	g_ValueOfAD[0] = (g_ValueOfAD[0] < 10? 10:g_ValueOfAD[0]);	//������ֵ�޷�
	g_ValueOfAD[1] = (g_ValueOfAD[1] < 10? 10:g_ValueOfAD[1]);
	g_ValueOfAD[2] = (g_ValueOfAD[2] < 10? 10:g_ValueOfAD[2]);
	g_ValueOfAD[3] = (g_ValueOfAD[3] < 10? 10:g_ValueOfAD[3]);
    g_ValueOfAD[4] = (g_ValueOfAD[4] < 10? 10:g_ValueOfAD[4]);

	g_fDirectionError[2] = (float)(g_ValueOfAD[0] - g_ValueOfAD[4])/(g_ValueOfAD[0] + g_ValueOfAD[4] + 2*g_ValueOfAD[2]);//ˮƽ��еĲ�Ⱥ���Ϊƫ��
	g_fDirectionError[2] = (g_fDirectionError[2]>= 1? 1:g_fDirectionError[2]);	//ƫ���޷�
	g_fDirectionError[2] = (g_fDirectionError[2]<=-1?-1:g_fDirectionError[2]);

    g_fDirection_dot[3] = g_fDirection_dot[2];
    g_fDirection_dot[2] = g_fDirection_dot[1];
    g_fDirection_dot[1] = g_fDirection_dot[0];
    g_fDirection_dot[0] = g_fDirectionError[2];
    g_fdot = g_fDirection_dot[0]+g_fDirection_dot[1]-g_fDirection_dot[2]-g_fDirection_dot[3];
    g_fLastdot += (g_fdot-g_fLastdot)/5;

	g_fDirectionError[1] = (float)(g_ValueOfAD[1] - g_ValueOfAD[3])/(g_ValueOfAD[1] + g_ValueOfAD[3]);//��ֱ��еĲ�Ⱥ���Ϊƫ��
	g_fDirectionError[1] = (g_fDirectionError[1]>= 1? 1:g_fDirectionError[1]);	//ƫ���޷�
	g_fDirectionError[1] = (g_fDirectionError[1]<=-1?-1:g_fDirectionError[1]);
//    g_fDirectionError[1] = RANGE_FLOAT(g_fDirectionError[1], -0.5, 0.5);

//    //�ǶȲ���
//    float temp = 0;
//    temp =(int16) 100/(0.94 - 0.0022*Angle);
//    temp /= 100;
//    if (Trans_Flag == 0)
//        g_fDirectionError[0] *= 1.4;
    /*�ǶȲ���*/
    Offset = Angle_Offset();

    DirFuzzy_P = Fuzzy_P(g_fDirectionError[2]*10,g_fLastdot*40);

    DirFuzzy_D = Fuzzy_D(g_fDirectionError[2]*10,g_fLastdot*40);


    PPWM = g_fDirectionError[2]*DirFuzzy_P;
    PPWM *= DSP;    // �ٶȲ���
    DPWM = E_Gyro*DirFuzzy_D;
    Direct_PWM = PPWM + DPWM;
    /*б���*/
    Dir_PWM = g_fDirectionError[1]*8000 + E_Gyro*0;
    Direct_PWM += 0*Dir_PWM;
    Direct_PWM = RANGE_FLOAT(Direct_PWM, -2000, 2000);

    //����Ϊ��������
    if((Star_Flag & 0x80)&&(Run_Flag==1))
    {
        Trans_Dir_Flag();// ��ȡ��������־λ
        switch (Round_State)
        {
        case 0: break;                         //�޻�
        case 1: break;                         //�л�
        case 2: Direct_PWM = Dir_PWM;break;    //�뻷
        case 3: break;                         //����
        case 4: Direct_PWM = Direct_PWM+Dir_PWM*0;break; //����
        }
        /*������״̬����*/
        if (0 == Round_State){
            /*�м��н������*/
#if 0    //���е�
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
#else   //�̶�ֵ
            if(g_ValueOfAD[2] > M_Round_Flag)
                M_Round_Count++;
            else
                M_Round_Count = 0;
#endif
            /*��ߵ�н������*/
            if (g_ValueOfAD[1] > Round_Flag)
                L_Round_Count++;
            else if ((g_ValueOfAD[1]<Round_Flag/2) || (g_ValueOfAD[3]<Round_Flag/2))
                L_Round_Count = 0;
            /*�ұߵ�н������*/
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
                Accle_X += E_Gyro/100;   //�����ǻ��ֵõ��Ƕ�
            Last_Gyro = E_Gyro;
        }
        if (4 == Round_State){
#if 1    //���е�
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
#else   //�̶�ֵ
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
        /*������״̬�б�*/
        if ((0==Round_State) && (M_Round_Count>=3) && (L_Round_Count||R_Round_Count)){   //�л���?��
            beep_on();
            M_Round_Count = 0;
            Round_State=1;
            if (L_Round_Count > R_Round_Count)
                RFlag = 1;
            else if (L_Round_Count < R_Round_Count)
                LFlag = 1;
        }
        if (1==Round_State){                                                         //ȷ��׼��
            if ((g_fDirectionError[1]>0&&LFlag) || (g_fDirectionError[1]<0&&RFlag))
                Round_State=2;
            else if (g_ValueOfAD[2] < M_Round_Flag/2)
                Round_State = 0;
        }
        if ((2==Round_State) && ((abs(Accle_X)>3000)&& (CONT>3000))){  //���뻷�����л�
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
        if ((3==Round_State) && (g_ValueOfAD[1]>Round_Flag || g_ValueOfAD[3]>Round_Flag)){ //��Ҫ������
            Round_State=4;
            beep_on();
        }
        if ((4==Round_State) && (M_Round_Count>1)){ //�ڶ���·���е�
            Round_State = 5;
            beep_off();
        }
        if ((5==Round_State) && (CONT>6000)){  //Զ�뻷��
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



/**********************************************��������*****************************************************
* ��������: void Read_ADC(void)
* �������: void
* �������: void
* ��    ��: ��ȡ������ADCֵ
* С    ��: BBO
* ��    ��: 2019/3/9
************************************************************************************************************/
void Read_ADC(void)
{
     int16  i,j,k,temp;
     int16  ad_valu[5][5],ad_valu1[5],ad_sum[5];
     int16  ValueOfADOld[5],ValueOfADNew[5];

     for(i=0;i<5;i++)
     {
         ad_valu[0][i]=ADC_Ave(AD1, ADC_12bit, 5);  		// AD1��������1
         ad_valu[1][i]=ADC_Ave(AD2, ADC_12bit, 5);     		// AD2��������2
         ad_valu[2][i]=ADC_Ave(AD3, ADC_12bit, 5);  		// AD3��������3
         ad_valu[3][i]=ADC_Ave(AD4, ADC_12bit, 5);     		// AD4��������4
         ad_valu[4][i]=ADC_Ave(AD5, ADC_12bit, 5);     		// AD4��������5
     }

/*=========================ð����������==========================*///�������ֵ����Сֵ
     for(i=0;i<5;i++)
     {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
              if(ad_valu[i][k] > ad_valu[i][k+1])        //ǰ��ıȺ���Ĵ�  ����н���
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              }
           }
        }
     }
/*===========================��ֵ�˲�=================================*/
     for(i=0;i<5;i++)    //���м�����ĺ�
     {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valu1[i] = ad_sum[i] / 3;
     }

	for(i=0;i<5;i++)            //����ֵ�и�λ������
	 {
	 	g_ValueOfAD[i] = (int16)(ad_valu1[i]/10*10);

		//�ɼ��ݶ�ƽ����ÿ�βɼ����仯40
		ValueOfADOld[i] = g_ValueOfADFilter[i];
		ValueOfADNew[i] = g_ValueOfAD[i];

		if(ValueOfADNew[i]>=ValueOfADOld[i])
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>50?(ValueOfADOld[i]+50):ValueOfADNew[i]);
		else
			g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-60?(ValueOfADOld[i]-60):ValueOfADNew[i]);
	 }
}