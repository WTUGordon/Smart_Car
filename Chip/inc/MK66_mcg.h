/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_mcg.h
 * @brief      MCG PLL驱动
 * @author     山外科技
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
    PLLUSR      ,  //自定义设置分频系数模式，直接加载 全局变量 mcg_div 的值
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

    //由于我们函数的时钟是已MHz为单位，不支持小数。
    //所以需要其他频率的朋友，请用自定义分频系数的方法，并自行修改代码

    PLL_MAX,
} PLL_e;
#endif



typedef struct
{
    uint16  clk;         //
    uint8   prdiv;       //外部晶振分频因子选项
    uint8   vdiv;        //外部晶振倍频因子选项
} mcg_cfg_t;

//时钟分频因子
typedef struct
{
    uint8 core_div;    //内核时钟分频因子
    uint8 bus_div;     //总线时钟分频因子
    uint8 flex_div;    //flex时钟分频因子
    uint8 flash_div;   //flash时钟分频因子
} mcg_div_t;


uint16 pll_init(PLL_e pll);

__RAMFUNC    void set_sys_dividers(uint32 outdiv1, uint32 outdiv2, uint32 outdiv3, uint32 outdiv4);

/********************************************************************/
#endif /* __MK60_MCG_H__ */
