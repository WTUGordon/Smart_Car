/**********************************************BBO的总工程*****************************************************
在此之前你要知道：
1、我们的单片机是MK60DN512ZVLQ10，用的山外（野火）库
2、工程所有的中断服务函数放在Interrupt.c
3、所有端口定义在PORT_cfg.h
4、APP为飞思卡尔底层驱动，不能动（端口声明除外）。
   Board为BBO所需外设驱动以及BBO所写文件。
   Chip为山外的K60库文件。
5、数据类型以及其他的宏定义在common.h
6、在一个.c文件写的变量或函数，需要在相应的.h文件加extern再次声明。才能在其他.c文件无需声明使用
7、变量定义规范，g_fCarAngle：g_全局，f数据类型(8位c，16位n，32位f，64位lf，无符号u)，
   CarAngle变量名称（单词首字母大写） <大部分变量声明并没有按照要求>
8、初始化函数都在Car_Init.c里面
9、include.h 放所有的头文件
10、5.19号更换为MK66FX。
11、核心在direction.c & control.c 这是你的重点
************************************************************************************************************/


#include "common.h"
#include "include.h"

uint8 Start_Flag=0;//起步标志
uint8 Speed_Select=3;//速度选择

/*         水平       直立
**   1:   1300      1300
**   2   1400      1300
**   3:   1500      1400
**   4:   1600      1500
**   5:   1700      1600
*/

int32 Speed_Horizon=0;  //水平速度
int32 Speed_Upright=0;   //直立速度
uint8 Trans_Switch=1;     //转化开关
uint8 Honwai_Option=0;//红外对管是否采用或 为1采用

void Display(void);

void main(void)
{
    Car_Init();
    Trans_Flag=0x00;

    Start_Flag=0;
    char txt1[16]=" ";
    while(Angle<250)
    {
        if(key_check(KEY_1) ==  KEY_DOWN)
            Speed_Select++;
        if(key_check(KEY_2) ==  KEY_DOWN)
            Speed_Select--;

        if(key_get(SW_4)==1)
            Trans_Switch=0;
        else
            Trans_Switch=1;
        
        if(key_get(SW_3)==1)
            Star_Flag = 0x7f;
        else
            Star_Flag = 0xff;
        
        if(key_get(SW_2)==1)
            Honwai_Option = 1;
        else
            Honwai_Option = 0;
        

        sprintf(txt1,"%05d",(int16)Angle);
        LCD_P6x8Str(60,1,(uint8 *)txt1);

        sprintf(txt1,"%05d",Speed_Select);
        LCD_P6x8Str(60,3,(uint8 *)txt1);

        if(Trans_Switch)
            LCD_P6x8Str(60,4,"Trans ON");
        else
            LCD_P6x8Str(60,4,"Trans OFF");
        
        if(Star_Flag&0x80)
            LCD_P6x8Str(60,5,"Huan ON");
        else
            LCD_P6x8Str(60,5,"Huan OFF");
        
        if(Honwai_Option)
            LCD_P6x8Str(60,6,"Honwai ||");
        else
            LCD_P6x8Str(60,6,"Honwai &&");
    }

    switch(Speed_Select)
    {
        case 1:
            Speed_Horizon=1300;
            Speed_Upright=1300;
            break;
        case 2:
            Speed_Horizon=1400;
            Speed_Upright=1300;
            break;
        case 3:
            Speed_Horizon=1500;
            Speed_Upright=1300;
            break;
        case 4:
            Speed_Horizon=1600;
            Speed_Upright=1400;
            break;
        case 5:
            Speed_Horizon=1700;
            Speed_Upright=1500;
            break;
        default:
            Speed_Horizon=1500;
            Speed_Upright=1300;
            break;
    }

    DELAY_MS(1000);
    LCD_CLS();

    Start_Flag=1;

    while(1)
    {

        camera_get_img();                                   //摄像头获取图像  采集一次8~12ms
        // pit_time_start(PIT1);
        //uint16 code_Time_flag =pit_time_get_us(PIT1)/1000;
        img_extract(img, imgbuff,CAMERA_SIZE);  //白255  黑0

        Img_Process(img);
        Display();

        /*起跑线*/
        if(GPIO_GET(REED)==0)
            led (LED2,LED_ON);
        else
            led (LED2,LED_OFF);

        if(GPIO_GET(REED)==0&&(Star_Flag&0x08)&&Run_Flag==1 )//低电平亮                    Break_Count>=4&&
        {
            DELAY_MS(250);
            Stop=1;
        }

        /*断路*/
        if(GPIO_GET(RPR1)==1)
            led (LED0,LED_ON);
        else
            led (LED0,LED_OFF);

        if(GPIO_GET(RPR2)==1)
            led (LED1,LED_ON);
        else
            led (LED1,LED_OFF);

        if(Honwai_Option)
        {
            if((GPIO_GET(RPR2)==1||GPIO_GET(RPR1)==1)&& (Star_Flag&0x20)&&Break_Flag==0)//高电平亮
            {
                if(Bar_State == 0 || Bar_State == 5)
                {
                    while(GPIO_GET(RPR2)==1);
                    Break_Flag=1;
                }
            }
        }
        else
        {
            if(GPIO_GET(RPR2)==1&&GPIO_GET(RPR1)==1&& (Star_Flag&0x20)&&Break_Flag==0)//高电平亮
            {
                if(Bar_State == 0 || Bar_State == 5)
                {
                    while(GPIO_GET(RPR2)==1);
                    Break_Flag=1;
                }
            }
        }
        if (Break_Flag == 1)
        {
            Break_Count++;
            if(Break_Count%2==1)
                Break_Flag++;
            else
                Break_Flag=0;
        }
        if (Break_Flag == 2)//检测到断路
        {
            if ((Trans_Flag & 0x01)) Trans_Flag = 0x05;
            else Trans_Flag = 0x02; //变形
            Break_Flag = 0;
        }

        switch(Up_State)
        {
            case 0:/*电感的*/  /*路障的*/  /*模糊的*/   /*圆环的*/ /*直立水平速度*/  /*坡道的*/ /*六轴的*/
                Send_data[0]=(int16)Direct_PWM;
                Send_data[1]=g_ValueOfAD[0];
                Send_data[2]=g_ValueOfAD[1];
                Send_data[3]=g_ValueOfAD[2];
                Send_data[4]=g_ValueOfAD[3];
                Send_data[5]=g_ValueOfAD[4];
                Send_data[6]=(int16)(g_fDirectionError[0]*1000);
                Send_data[7]=(int16)(g_fDirectionError_dot*10000);
                send_oscilloscope((uint8 *)Send_data, sizeof(Send_data));
                break;

            case 1:/*路障的*/
                Send_data[0]=(int16)Direct_PWM;
                Send_data[1]=Accle_x;
                Send_data[2]=E_Gyro;
                Send_data[3]=Cont;
                Send_data[4]=Barrier_Distance;
                Send_data[5]=Bar_State*1000;
                Send_data[6]=Pulse_Cont;
                Send_data[7]=g_ValueOfAD[4];
                send_oscilloscope((uint8 *)Send_data, sizeof(Send_data));
                break;

            case 2: /*模糊的*/
                Send_data[0]=(int16)(PPWM);
                Send_data[1]=E_Gyro;
                Send_data[2]=(int16)DirFuzzy_P;
                Send_data[3]=(int16)(DirFuzzy_D*100000);
                Send_data[4]=(int16)(DPWM);
                Send_data[5]=(int16)(g_fDirectionError[2]*10000);
                Send_data[6]=(int16)(g_fLastdot*10000);
                Send_data[7]=(int16)Direct_PWM;
                send_oscilloscope((uint8 *)Send_data, sizeof(Send_data));
                break;

            case 3:   /*圆环的*/
                Send_data[0]=Accle_X;
                Send_data[1]=g_ValueOfAD[1];
                Send_data[2]=g_ValueOfAD[2];
                Send_data[3]=g_ValueOfAD[3];
                Send_data[4]=Round_State*1000;
                Send_data[5]=(int16)(CONT);
                Send_data[6]=(int16)(g_fDirectionError[1]*1000);
                Send_data[7]=(int16)Direct_PWM;
                send_oscilloscope((uint8 *)Send_data, sizeof(Send_data));
                break;

            case 4:  /*直立水平速度*/
                Send_data[0]=(int16)Gyro_z;
                Send_data[1]=PWM;
                Send_data[2]=Angle_Tar;
                Send_data[3]=(int16)Angle;
                Send_data[4]=(int16)Speed_Out;
                //Send_data[5]=(int16)Angle-Angle_Tar;
                Send_data[6]=(int16)Speed_Set;//ACC_RealZ;// Speed_Set
                Send_data[7]=(int16)Speed[3];//Speed[4];
                send_oscilloscope((uint8 *)Send_data, sizeof(Send_data));
                break;

            case 5: /*坡道的*/
                Send_data[0]=g_ValueOfAD[0];
                Send_data[1]=g_ValueOfAD[2];
                Send_data[2]=g_ValueOfAD[4];
                Send_data[3]=Slope_State*1000;
                Send_data[4]=(int16)Angle;
                Send_data[5]=PWM;
                Send_data[6]=Gyro_z;
                Send_data[7]=Speed[0];
                send_oscilloscope((uint8 *)Send_data, sizeof(Send_data));
                break;

            case 6:/*六轴的*/
                Send_data[0]=(int16)ACC_Real.X;
                Send_data[1]=(int16)ACC_Real.Y;
                Send_data[2]=(int16)ACC_Real.Z;
                Send_data[3]=(int16)GYRO_Real.X;
                Send_data[4]=(int16)GYRO_Real.Y;
                Send_data[5]=(int16)GYRO_Real.Z;
                send_oscilloscope((uint8 *)Send_data, sizeof(Send_data));
                break;

            default:
                break;
        }
        

    }
}


/*************************************************************************************************************
函数名称: Display
功    能: 通过拨码盘进行菜单选择
参    数:
返    回: void
**************************************************************************************************************/
void Display(void)
{
    char txt[16]=" ";

    uint8 Boma=0;
    Boma |= key_get(SW_4)<<3;
    Boma |= key_get(SW_3)<<2;
    Boma |= key_get(SW_2)<<1;
    Boma |= key_get(SW_1)<<0;
     if(symbol)
        LCD_P6x8Str(90,0,"+");
    else
        LCD_P6x8Str(90,0,"-");
    sprintf(txt,"%02d",Up_State);
    LCD_P6x8Str(100,0,(uint8 *)txt);

    switch(Boma)
    {
        case 0x01:
            dis_bmp(60,80,img,0x7F);

            sprintf(txt,"%05d",Barrier_Distance);
            LCD_P6x8Str(90,1,(uint8 *)txt);
            sprintf(txt,"%05d",Emit_Flag);
            LCD_P6x8Str(90,2,(uint8 *)txt);
            sprintf(txt,"%05.1f",Dolly_now_angle);
            LCD_P6x8Str(90,3,(uint8 *)txt);
            sprintf(txt,"%05d ",(int16)Angle);
            LCD_P6x8Str(90,4,(uint8 *)txt);

            break;

        case 0x02:
            sprintf(txt,"%05d",g_ValueOfAD[0]);
            LCD_P6x8Str(0,1,(uint8 *)txt);
            sprintf(txt,"%05d",g_ValueOfAD[1]);
            LCD_P6x8Str(0,2,(uint8 *)txt);
            sprintf(txt,"%05d",g_ValueOfAD[2]);
            LCD_P6x8Str(0,3,(uint8 *)txt);
            sprintf(txt,"%05d",g_ValueOfAD[3]);
            LCD_P6x8Str(0,4,(uint8 *)txt);
            sprintf(txt,"%05d",g_ValueOfAD[4]);
            LCD_P6x8Str(0,5,(uint8 *)txt);




            sprintf(txt,"%05.2f",g_fDirectionError[0]);
            LCD_P6x8Str(50,1,(uint8 *)txt);

            sprintf(txt,"%05.2f",g_fDirectionError[1]);
            LCD_P6x8Str(50,2,(uint8 *)txt);

            sprintf(txt,"%05d",Speed[4]);
            LCD_P6x8Str(50,4,(uint8 *)txt);

            sprintf(txt,"%05d",(int16)Direct_PWM);
            LCD_P6x8Str(50,6,(uint8 *)txt);

            break;

        case 0x04:

             LCD_P6x8Str(0,1,"IRD");
            LCD_P6x8Str(0,2,"SpdSet");
            LCD_P6x8Str(0,3,"Speed");

            LCD_P6x8Str(0,4,"KP");
            LCD_P6x8Str(0,5,"KI");
            LCD_P6x8Str(0,6,"KD");


//         sprintf(txt0,"%05X",Data_IRD);
//        LCD_P6x8Str(40,0,(uint8 *)txt0);

//            sprintf(txt,"%05.0f",g_dirControl_P);
//            LCD_P6x8Str(80,4,(uint8 *)txt);
//
//            LCD_P6x8Str(80,5,"Spedir");
//
//            sprintf(txt,"%05.4f",g_dirControl_D);
//            LCD_P6x8Str(80,6,(uint8 *)txt);
            break;

        case 0x08:
            LCD_P6x8Str(0,1,"GySpS");
            LCD_P6x8Str(0,2,"AngSp");
            LCD_P6x8Str(0,3,"pwmLR");
            LCD_P6x8Str(0,4,"KP");
            LCD_P6x8Str(0,5,"KI");
            LCD_P6x8Str(0,6,"KD");

            sprintf(txt,"%05d",Gyro_z);
            LCD_P6x8Str(40,1,(uint8 *)txt);
            sprintf(txt,"%05d ",(int16)Angle);
            LCD_P6x8Str(40,2,(uint8 *)txt);

            sprintf(txt,"%05d",Speed_Set);
            LCD_P6x8Str(80,1,(uint8 *)txt);
            sprintf(txt,"%05d",Speed[0]);
            LCD_P6x8Str(80,2,(uint8 *)txt);

            sprintf(txt,"%05d ",PWM_L);
            LCD_P6x8Str(40,3,(uint8 *)txt);
            sprintf(txt,"%05d ",PWM_R);
            LCD_P6x8Str(80,3,(uint8 *)txt);

            sprintf(txt,"%05.2f",Speed_PID.KP);
            LCD_P6x8Str(30,4,(uint8 *)txt);
            sprintf(txt,"%05d",Trans_Flag);
            LCD_P6x8Str(30,5,(uint8 *)txt);
            sprintf(txt,"%05.3f",Speed_PID.KD);
            LCD_P6x8Str(30,6,(uint8 *)txt);

            sprintf(txt,"%05.3f",Angle_PID.KP);
            LCD_P6x8Str(80,4,(uint8 *)txt);
            sprintf(txt,"%05.4f",Angle_PID.KI);
            LCD_P6x8Str(80,5,(uint8 *)txt);
            sprintf(txt,"%05.4f",Angle_PID.KD);
            LCD_P6x8Str(80,6,(uint8 *)txt);

            break;

        default:
            LCD_CLS();
    }

}
