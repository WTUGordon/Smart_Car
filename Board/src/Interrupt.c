/******************************************************
* by          Gordon
* 时间        2019/3/6
* 文件名      Interrupt.c
* 内容        存放工程的所有中断服务函数
* 软件        IAR7.3
* 单片机      MK60DN512ZVLQ10

******************************************************/

#include  "include.h"
#include  "Interrupt.h"

/*全局变量声明*/

uint8 Trans_Flag=0x00;
//  0x04直立变水平中   0x02水平变直立中   0X00直立  0x01水平

uint8 Star_Flag = 0xff;
//是否开启道路元素识别 1开启，0关闭
//  圆环   路障    断路    斜坡 | 起跑线 出赛道保护 堵转保护 疯转停车
//   8   |   4   |   2   |   1   |         8   |   4   |   2   |   1   |

int8 Up_State =3;
//0电感 1路障 2模糊 3圆环 4直立水平速度 5坡道 6六轴的 7不发送*/

int32 PWM=0;   //输出速度PWM


int32 Last_Speed = 0;  //路障距离
uint8  Count_Control=0;  //PIT1定时器计数
int16 Send_data[8];     //发送至上位机的数组
uint8 Speed_Flag=0;   //速度控制标志
uint16 Barrier_Distance=0; //与路障距离
uint16 Last_Barrier_Distance=0 ;   //上一次路障距离
uint8 Data_IRD = 0;  //红外码值
uint8  symbol = 1; //参数加减设置  1为加 0为减
uint8 Bar_State=0;  //路障状态标志 0未遇路障，1开始打角，2开始回正，3开始反打角，4完全回正,5通过路障
uint8 Break_Flag=0;//断路标志
uint8 Break_Count=0; //断路标志计数
uint8 Bar_Judge=0;//路障判断次数

uint8 Timer_900ms=0;
uint16 Timer_60ms=0;

int32 Angle_Zero=0;//小车零点

extern uint8 Start_Flag;//起步标志
extern uint8 Trans_Switch;
extern int32 Speed_Horizon;
extern int32 Speed_Upright;

/*红外中断处理*/
void PORTD_IRQHandler(void)//PTD6
{
    uint8  n = 6;    //引脚号
    uint8  data=0;

    if(PORTD_ISFR & (1 << n))           //PTD7 触发中断
    {
        PORTD_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */

        data = decode();
        if(data!=0)     Data_IRD=data;
        switch(Data_IRD)
        {
        case 0x47:
            symbol = !symbol; break;
        case 0x46:
            if((Trans_Flag&0x01)==0x01) Trans_Flag=0x05;
            else Trans_Flag=0x02; break;
        case 0x45:
            Stop = !Stop; break;

        case 0x44:
            Up_State += 1;
            if(Up_State>7)
                Up_State=7;
            break;
        case 0x07:
            Up_State -= 1;
            if(Up_State<0)
                Up_State=0;
            break;
//        case 0x44:
//            Angle_x+=2000;
//            break;
//        case 0x07:
//            Angle_x-=2000;
//            break;

        case 0x40:
            Angle_Tar += 50; break;
        case 0x15:
            Angle_Tar -= 50; break;
        case 0x43:
            Speed_Set += 1000; break;
        case 0x09:
            Speed_Set -= 1000; break;

//        case 0x16:
//                if(symbol)  Gyro_PID.KP += 0.05;
//                else    Gyro_PID.KP -= 0.05;
//                break;
//        case 0x19:
//            if(symbol)  Gyro_PID.KI += 0.0001;
//            else    Gyro_PID.KI -= 0.0001;
//            break;
//        case 0x0d:
//            if(symbol)  Gyro_PID.KD += 0.0005;
//            else    Gyro_PID.KD -= 0.0005;
//            break;

        case 0x0c:
            if(symbol)  Angle_PID.KP +=0.2;
            else    Angle_PID.KP -=0.2;
            break;
        case 0x18:
            if(symbol)  Angle_PID.KI +=0.0001;
            else    Angle_PID.KI -=0.0001;
            break;
        case 0x5e:
            if(symbol)   Angle_PID.KD +=0.02;
            else     Angle_PID.KD -=0.02;
            break;

        case 0x08:
            if(symbol)   Speed_PID.KP +=0.02;
            else     Speed_PID.KP -=0.02;
            break;
        case 0x1c:
            if(symbol)   Speed_PID.KI +=0.0005;
            else     Speed_PID.KI -=0.0005;
            break;
        case 0x5a:
            if(symbol)   Speed_PID.KD +=0.01;
            else     Speed_PID.KD -=0.01;
            break;

//        case 0x42:
//            if(symbol)   Direction_PD[1][0] +=50;
//            else     Direction_PD[1][0] -=50;
//            break;
//        case 0x4a:
//            if(symbol)   Direction_PD[1][1] +=0.001;
//            else     Direction_PD[1][1] -=0.001;
//            break;

//        case 0x52:
//            if(symbol)   Gyro_Rate +=0.01;
//            else     Gyro_Rate -=0.01;
//            break;

        default:
            Data_IRD=0;
        }

        Data_IRD=0;
        /* 以上为用户任务  */
    }
}


void PORTE_IRQHandler(void)//PTE4
{
    uint8  n = 4;    //引脚号

    if(PORTE_ISFR & (1 << n))           //PTE4 触发中断
    {
        PORTE_ISFR  = (1 << n);        //写1清中断标志位
        if(Star_Flag & 0x40)  //开启了路障检测
        {
            if(gpio_get(ECHO)==1)
                pit_time_start(PIT3);
            else
            {
                uint32 time = pit_time_get_us(PIT3);
                Barrier_Distance=(uint16)(time*0.17f);// mm
                Barrier_Distance=RANGE_UINT16(Barrier_Distance,1,4000);
                if(Barrier_Distance<=1200&&Barrier_Distance>500)  //1.2m内
                {
                    Timer_900ms=0;
                    Timer_60ms=0;
                    Bar_Judge=0;
                    Bar_State=1;
                    Emit_Flag=0;
                    beep_on();
                    Star_Flag &= (~0x40); //关闭路障检测
                }
                if(Bar_State!=1&&Barrier_Distance>1500)
                {
//                    Bar_Judge++;
//                    if(!Timer_900ms&&Bar_Judge>=2) //在900ms内两次触发则必有路障
//                          Emit_Flag=1;
//                    else
                         Emit_Flag=0;

                }
            }
        }
        /* 以上为用户任务  */
    }
}

/*摄像头*/
void PORTA_IRQHandler(void)
{
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    if(flag & (1 << 29))                                 //PTA29触发中断
    {
        camera_vsync();
    }
}

/*PIT0中断服务函数*/
void PIT0_IRQHandler(void) //60ms
{
    PIT_Flag_Clear(PIT0);       //清中断标志位
    if(Emit_Flag)
    {
        gpio_set(TRIG,1);
        DELAY_US(20);
        gpio_set(TRIG,0);
    }
}


/*PIT1中断服务函数*/
void PIT1_IRQHandler(void)     //2ms
{
    PIT_Flag_Clear(PIT1);       //清中断标志位
    Count_Control++;

    /*方向环6ms*/
    if(Count_Control%2==0)
    {

        Direction_Control();
        if ((Bar_State>0) && (Bar_State < 5))
        {
            Dispose_Barrier();    /*路障处理*/
        }


       if(Run_Flag==1)
       {
            if(Angle>400||Speed[0]<0)
                Stop=1;
            if(!Trans_Switch)
            {
                Trans_Flag = 0x02;
                Star_Flag &= (~0x20); //关闭断路检测
                Trans_Switch=1;
            }

//            if(Star_Flag&0x10)
                //Slope();
       }
    }

    /*速度环60ms*/
    if (Count_Control == 20)
    {
        Count_Control=0;
        if(Bar_Judge)
            Timer_60ms++;
        if(Timer_60ms==15)
        {
            Timer_60ms=0;
            Timer_900ms=1;
        }
        if(Start_Flag)
            Speed_Flag =1;
    }
    if(Timer_900ms&&Bar_Judge==1)//900ms内路障未判断成功
    {
        Bar_Judge=0;
        Emit_Flag=0;
        Timer_900ms=0;
        Timer_60ms=0;
    }

    //Trans_Flag=0x01;
    if(GPIO_GET(REED)==0&&(Star_Flag&0x08)&&Run_Flag==1 )//低电平亮                    Break_Count>=4&&
    {
//        DELAY_MS(250);
        Stop=1;
    }

    switch(Trans_Flag)
    {
        case 0x00: //直立
            Speed_Set=Speed_Upright;
            Angle_Zero=ANGLE_SET;
            break;
        case 0x02://直立切换成水平  //角度3ms 速度60ms
            Speed_Set=Speed_Upright;
            Angle_Zero -=8;
            if(Angle_Zero<HORIZON_SET)
            {
                Angle_Zero=HORIZON_SET;
                Trans_Flag=0x01;
            }
            break;
        case 0x01://水平
            Angle_Zero=Speed_Horizon;
            break;
        case 0x05://水平切换成直立
            Speed_Set=Speed_Horizon-200;
            Angle_Zero+=4;
            if(Angle_Zero>ANGLE_SET)
            {
                Angle_Zero=ANGLE_SET;
                Trans_Flag=0x00;
            }
            break;
        default:
            //Trans_Flag=0x00;
            break;
    }

    PWM=Balance_Control(Angle_Zero);

    MotorSpeedOut(PWM,-Direct_PWM);   //  后面一项为正 向左转          -Direct_PWM
//     MotorSpeedOut(1200,0);   //  后面一项为正 向左转

}

/**/
int32 Accle_x;
float Bar_P=0.5;//0.5;
float Bar_D=0.12;//0.12;
int16 Last_Gyro = 0;
int16 Angle_Er = 0;
int16 LastAngle_Er = 0;
/*
输入：目标方向角度
功能：处理路障方向PID
By  ：Gordon
*/
void PID_Barrier(int16 Aim_Angle)
{
    Angle_Er = Accle_x - Aim_Angle;
    Direct_PWM = Bar_P*Angle_Er + E_Gyro*Bar_D;
    Direct_PWM = RANGE_FLOAT(Direct_PWM,-1500,1500);    //限幅，防止角速度过大。

}

/*
功能：路障处理
By  ：Gordon
*/
int32 Pulse_Cont = 0;
int32 Cont;
uint16 Dis_Barrier = 3000;
int16 Ang_Barrier = 8000;
int16 Dis_Barrier_Back = 3000;
int16 M_Barrier_Count = 0;
int16 BarSpeed = 0;
void Dispose_Barrier(void)
{
    uint8 Distance_Ratio=0;
    if ((Trans_Flag & 0x01) == 0x01)//水平状态
	{
        Distance_Ratio=5;
		Dis_Barrier = 2500;
        Ang_Barrier = 8000;
        Dis_Barrier_Back = 2500;
	}
	else //直立
	{
        Distance_Ratio=4;
		Dis_Barrier = 4000;
        Ang_Barrier = 6000;
        Dis_Barrier_Back=4000;
	}
    /*计算距离*/
    BarSpeed = FTM_QUAD_get(FTM1) + FTM_QUAD_get(FTM2);
    Pulse_Cont += (BarSpeed - Last_Speed);
    Cont += Pulse_Cont/10;
    Last_Speed = BarSpeed;
    /*滤波，算角度*/
    E_Gyro =(int16)E_Gyro + (E_Gyro-Last_Gyro)/10;
    if(Last_Gyro)
        Accle_x += E_Gyro/100;   //陀螺仪积分得到角度
    Last_Gyro = E_Gyro;
    switch (Bar_State)
    {
    case 1: PID_Barrier(Ang_Barrier);break;                  //外打角
    case 2: PID_Barrier(0);break;                           //外回正
    case 3: PID_Barrier(-Ang_Barrier);break;                //内打角
    case 4: PID_Barrier(0);break;                          //内回正
    }
    if ((Cont > Dis_Barrier) && (Bar_State == 1))  //外打角结束，开始外回正
    {
        Cont = 0;
        Bar_State = 2;
        beep_off();
    }
    if ((Cont > Barrier_Distance*Distance_Ratio) && (Bar_State == 2)) //外回正，开始内打角
    {
        Cont = 0;
        Bar_State = 3;
        beep_on();
    }
    if ((Cont > Dis_Barrier_Back) && (Bar_State == 3)) //进入赛道了，开始内回正
    {
        Cont = 0;
        Bar_State = 4;
        beep_off();
    }
    if ((Accle_x > -1000) && (Bar_State == 4)) //内回正了，开始正常行驶。
    {
        Cont = 0;
        Bar_State = 5;
        Accle_x = 0;
        Last_Gyro = 0;
        Last_Speed = 0;
        beep_off();
        Stop=0;
    }
}


void DMA0_IRQHandler()
{
    camera_dma();
}