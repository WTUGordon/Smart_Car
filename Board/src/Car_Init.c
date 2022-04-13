/*
*。小车初始化文件
*。BBO
*。Gordon
*。3/13
*/

#include "common.h"
#include "include.h"


extern float Gyro_Vel[4]  ,Angle_Vel[4], Speed_Vel[4] ;
void Car_Init(void)
{
    DisableInterrupts;
    /*设置中断优先级*/
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4); //PriorityGroup = NVIC_PriorityGroup_4表示 4bit 抢占优先级  ，0bit 亚优先级
    NVIC_SetPriority(PORTE_IRQn,0);  //超声波
    NVIC_SetPriority(PORTD_IRQn,1);  // 红外
    NVIC_SetPriority(PORTA_IRQn,2); //场中断
    NVIC_SetPriority(DMA0_DMA16_IRQn,3); //摄像头
    NVIC_SetPriority(PIT1_IRQn,4);  //平衡
    NVIC_SetPriority(PIT0_IRQn,5);//超声波

    /*PID参数初始化*/
//    PID_Parameter_Init(&Gyro_PID,Gyro_Vel);
    PID_Parameter_Init(&Angle_PID,Angle_Vel);
    PID_Parameter_Init(&Speed_PID,Speed_Vel);
    PID_Parameter_Init(&Horizon_PID,Horizon_Vel);
     /*蜂鸣器初始化打开*/
    beep_init();
//    beep_on();

      /*干簧管*/
    gpio_init(REED,GPI,0);
    port_init_NoALT(REED, PULLUP);         //保持复用不变，仅仅改变配置选项

    /*断路识别*/
    gpio_init(RPR1,GPI,0);
    gpio_init(RPR2,GPI,0);
    port_init_NoALT(RPR1, PULLUP);         //保持复用不变，仅仅改变配置选项
    port_init_NoALT(RPR2, PULLUP);         //保持复用不变，仅仅改变配置选项

    /*  超声波*/
    gpio_init(TRIG,GPO,0);    //初始化发送端
    gpio_init(ECHO,GPI,0);
    port_init (ECHO, IRQ_EITHER | PF | ALT1| PULLDOWN);     //跳变沿触发
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);    //设置PORTE的中断复位函数为 PORTE_IRQHandler

    /*LED初始*/
    led_init(LED_MAX);
    led(LED0,LED_OFF);

    /*OLED初始*/
    LCD_Init();

    /*蓝牙串口初始*/
    uart_init (UART4, 115200);

    /*九轴初始化*/
    IIC_Init();
    Init8700();  //加速度
    Init2100();  //陀螺仪
//    while (GYRO_Real.GyrooffsetOK != 1)
//    {
//        Get_Offset();
//    }

    /*FTM初始设置*/
    FTM_QUAD_Init(FTM1);
    FTM_QUAD_Init(FTM2);
    FTM_PWM_init(FTM0,FTM_CH0,14000,0);//右前死区10
    FTM_PWM_init(FTM0,FTM_CH1,14000,0);//右后死区10
    FTM_PWM_init(FTM0,FTM_CH2,14000,0);//左后死区10
    FTM_PWM_init(FTM0,FTM_CH3,14000,0);//左前死区10

    /*电磁AD初始*/
    adc_init(AD1);
    adc_init(AD2);
    adc_init(AD3);
    adc_init(AD4);
    adc_init(AD5);

    /*OV7725初始化*/
    camera_init(imgbuff);

    /*OV7725的DMA、引脚中断*/
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置LPTMR的中断复位函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_DMA16_VECTORn ,DMA0_IRQHandler);      //设置LPTMR的中断复位函数为 DMA0_IRQHandler

    /*红外解码处理*/
    port_init(PTD6, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTD6 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
    set_vector_handler(PORTD_VECTORn ,PORTD_IRQHandler);    //设置PORTE的中断复位函数为 PORTE_IRQHandler

    /*编码器中断数据处理，速度调整*/
    pit_init_ms(PIT0, 60);                                  //初始化PIT0，定时时间为： 60ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断复位函数为 PIT0_IRQHandler

    /*九轴中断数据处理、角度调整*/
    pit_init_ms(PIT1, 3);                                   //初始化PIT1，定时时间为： 2ms
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT1的中断复位函数为 PIT1_IRQHandler

    /*按键初始化*/
    key_init(KEY_MAX); //按键PTC0与LED1冲突  PTB16与编码器FTM2冲突

    /*初始化结束，蜂鸣器关闭*/
    beep_off();

    enable_irq (PORTE_IRQn); //超声波
    enable_irq (PORTD_IRQn); //红外
    enable_irq (PIT0_IRQn);     //超声波 发送
    enable_irq (PIT1_IRQn);     //平衡

    EnableInterrupts;
}