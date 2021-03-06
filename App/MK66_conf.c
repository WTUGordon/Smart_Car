/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK66_conf.c
 * @brief      山外K66 平台配置功能实现文件
 * @author     山外科技
 * @version    v6.0
 * @date       2017-11-04
 */

#include    "common.h"
#include    "include.h"






void soft_delay_us(uint32_t us)
{
    //本函数是软件延时，不准的，仅仅是大概
    volatile  uint32_t i=us,j;

    while(i--)
    {
        j=0x1000;
        while(j--);
    }

}



void soft_delay_ms(uint32_t ms)
{
    //本函数是软件延时，不准的，仅仅是大概
    volatile  uint32_t i=ms;

    while(i--)
    {
        soft_delay_us(1000);
    }

}

/*!
 *  @brief      断言失败所执行的函数
 *  @param      file    文件路径地址
 *  @param      line    行数
 *  @since      v5.0
 *  Sample usage:       assert_failed(__FILE__, __LINE__);
 */
const char ASSERT_FAILED_STR[] = "Assertion failed in %s at line %d\n";

void assert_failed(char *file, int line)
{
    led_init(LED0);
	FTM_PWM_Duty(FTM0, FTM_CH0, 0);
	FTM_PWM_Duty(FTM0, FTM_CH1, 0);
	FTM_PWM_Duty(FTM0, FTM_CH2, 0);
	FTM_PWM_Duty(FTM0, FTM_CH3, 0);
    while (1)
    {

        DEBUG_PRINTF(ASSERT_FAILED_STR, file, line);      //通过串口提示断言失败

        //死循环等待程序员检测为何断言失败
        led_turn(LED0);
        soft_delay_ms(1000);
        
    }
}

/*!
 *  @brief      重定义printf 到串口
 *  @param      ch      需要打印的字节
 *  @param      stream  数据流
 *  @since      v5.0
 *  @note       此函数由printf所调用
 */
int fputc(int ch, FILE *stream)
{
    uart_putchar(VCAN_PORT, (char)ch);
    return(ch);
}

/*!
 *  @brief      默认中断服务函数
 *  @since      v5.0
 *  @note       此函数写入中断向量表里，不需要用户执行
 */
void default_isr(void)
{


#ifdef  DEBUG_MODE
#define VECTORNUM    ((SCB_ICSR & SCB_ICSR_VECTACTIVE_MASK)>>SCB_ICSR_VECTACTIVE_SHIFT)
                            //等效于 (*(volatile uint8_t*)(0xE000ED04))
    uint8 vtr = VECTORNUM;


    led_init(LED1);
	FTM_PWM_Duty(FTM0, FTM_CH0, 0);
	FTM_PWM_Duty(FTM0, FTM_CH1, 0);
	FTM_PWM_Duty(FTM0, FTM_CH2, 0);
	FTM_PWM_Duty(FTM0, FTM_CH3, 0);
    while(1)
    {
        led_turn(LED1);
        DEBUG_PRINTF("\n****default_isr entered on vector %d*****\n\n", vtr);

        DELAY_MS(1000);
    }
#else
    return;
#endif
}
