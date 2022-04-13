/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_mcg.h
 * @brief      MCG PLL����
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-29
 */


#ifndef __MK66_MCG_H__
#define __MK66_MCG_H__

#include "common.h"

/********************************************************************/
#if defined(MK66F18)
typedef enum
{
    PLLUSR      ,  //�Զ������÷�Ƶϵ��ģʽ��ֱ�Ӽ��� ȫ�ֱ��� mcg_div ��ֵ
    PLL80       ,
    PLL88       ,
    PLL96       ,
    PLL100       ,
    PLL120       ,
    PLL140       ,
    PLL152       ,
    PLL160       ,
    PLL180       ,
    PLL188       ,
    PLL200       ,
    PLL224       ,
    PLL232       ,
    PLL240       ,
    PLL248       ,
    PLL256       ,
    PLL264       ,

    //�������Ǻ�����ʱ������MHzΪ��λ����֧��С����
    //������Ҫ����Ƶ�ʵ����ѣ������Զ����Ƶϵ���ķ������������޸Ĵ���

    PLL_MAX,
} PLL_e;
#endif



typedef struct
{
    uint16  clk;         //
    uint8   prdiv;       //�ⲿ�����Ƶ����ѡ��
    uint8   vdiv;        //�ⲿ����Ƶ����ѡ��
} mcg_cfg_t;

//ʱ�ӷ�Ƶ����
typedef struct
{
    uint8 core_div;    //�ں�ʱ�ӷ�Ƶ����
    uint8 bus_div;     //����ʱ�ӷ�Ƶ����
    uint8 flex_div;    //flexʱ�ӷ�Ƶ����
    uint8 flash_div;   //flashʱ�ӷ�Ƶ����
} mcg_div_t;


uint16 pll_init(PLL_e pll);

__RAMFUNC    void set_sys_dividers(uint32 outdiv1, uint32 outdiv2, uint32 outdiv3, uint32 outdiv4);

/********************************************************************/
#endif /* __MK60_MCG_H__ */
