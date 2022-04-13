/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_mcg.c
 * @brief      MCG PLL驱动
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-29
 */


#include "common.h"
#include "MK66_mcg.h"


#if defined(MK66F18)

mcg_cfg_t mcg_cfg[PLL_MAX] =
{
    {0, 0, 0}, //PLLUSR 不使用
    {80,    1 ,     4       },                          //PLL80
    {88,    1 ,     6       },                          //PLL88
    {96,    1 ,     8       },                          //PLL96
    {100,   1 ,     9       },                          //PLL100
    {120,   1 ,     14       },                          //PLL120
    {140,   1 ,     19       },                          //PLL140
    {152,   1 ,     22       },                          //PLL152
    {160,   1 ,     24       },                          //PLL160
    {180,   4 ,     20       },
    //{180,   1 ,     29       },                          //PLL180
    {188,   1 ,     31       },                          //PLL188
    {200,   4 ,     24       },
    //{200,   0 ,     9       },                          //PLL200
    {224,   0 ,     12       },                          //PLL224
    {232,   0 ,     13       },                          //PLL232
    {240,   0 ,     14       },                          //PLL240
    {248,   0 ,     15       },                          //PLL248
    {256,   0 ,     16       },                          //PLL256
    {264,   0 ,     17       },                          //PLL264

};
#endif
mcg_div_t mcg_div = {CORE_DIV, BUS_DIV, FLEX_DIV, FLASH_DIV};


/*!
 *  @brief      MCG 分频配置
 *  @param      PLL_e       频率设置参数
 *  @since      v5.0
 */
void mcg_div_count(PLL_e pll)
{
    uint16 clk = mcg_cfg[pll].clk;
    if( (pll > PLLUSR) && (pll < PLL_MAX) )
    {
        //设置分频
        mcg_div.core_div    = 0;                                        // core = MCG

        if     (clk <= 1 * MAX_BUS_CLK)   mcg_div.bus_div = 0;          // bus  = MCG
        else if(clk <= 2 * MAX_BUS_CLK)   mcg_div.bus_div = 1;          // bus  = MCG/2
        else if(clk <= 3 * MAX_BUS_CLK)   mcg_div.bus_div = 2;          // bus  = MCG/3
        else if(clk <= 4 * MAX_BUS_CLK)   mcg_div.bus_div = 3;          // bus  = MCG/4
        else if(clk <= 5 * MAX_BUS_CLK)   mcg_div.bus_div = 4;          // bus  = MCG/5
        else if(clk <= 6 * MAX_BUS_CLK)   mcg_div.bus_div = 5;          // bus  = MCG/6
        else if(clk <= 7 * MAX_BUS_CLK)   mcg_div.bus_div = 6;          // bus  = MCG/7
        else if(clk <= 8 * MAX_BUS_CLK)   mcg_div.bus_div = 7;          // bus  = MCG/8
        else if(clk <= 9 * MAX_BUS_CLK)   mcg_div.bus_div = 8;          // bus  = MCG/9
        else if(clk <= 10 * MAX_BUS_CLK)   mcg_div.bus_div = 9;         // bus  = MCG/10
        else if(clk <= 11 * MAX_BUS_CLK)   mcg_div.bus_div = 10;        // bus  = MCG/11
        else if(clk <= 12 * MAX_BUS_CLK)   mcg_div.bus_div = 11;        // bus  = MCG/12
        else if(clk <= 13 * MAX_BUS_CLK)   mcg_div.bus_div = 12;        // bus  = MCG/13
        else if(clk <= 14 * MAX_BUS_CLK)   mcg_div.bus_div = 13;        // bus  = MCG/14
        else if(clk <= 15 * MAX_BUS_CLK)   mcg_div.bus_div = 14;        // bus  = MCG/15
        else                              mcg_div.bus_div = 15;         // bus  = MCG/16

        if     (clk <= 1 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 0;     // flex bus  = MCG
        else if(clk <= 2 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 1;     // flex bus  = MCG/2
        else if(clk <= 3 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 2;     // flex bus  = MCG/3
        else if(clk <= 4 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 3;     // flex bus  = MCG/4
        else if(clk <= 5 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 4;     // flex bus  = MCG/5
        else if(clk <= 6 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 5;     // flex bus  = MCG/6
        else if(clk <= 7 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 6;     // flex bus  = MCG/7
        else if(clk <= 8 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 7;     // flex bus  = MCG/8
        else if(clk <= 9 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 8;     // flex bus  = MCG/9
        else if(clk <= 10 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 9;    // flex bus  = MCG/10
        else if(clk <= 11 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 10;   // flex bus  = MCG/11
        else if(clk <= 12 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 11;   // flex bus  = MCG/12
        else if(clk <= 13 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 12;   // flex bus  = MCG/13
        else if(clk <= 14 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 13;   // flex bus  = MCG/14
        else if(clk <= 15 * MAX_FLEXBUS_CLK)   mcg_div.flex_div = 14;   // flex bus  = MCG/15
        else                                  mcg_div.flex_div = 15;    // flex bus  = MCG/16

        if     (clk <= 1 * MAX_FLASH_CLK)   mcg_div.flash_div = 0;      // flash  = MCG
        else if(clk <= 2 * MAX_FLASH_CLK)   mcg_div.flash_div = 1;      // flash  = MCG/2
        else if(clk <= 3 * MAX_FLASH_CLK)   mcg_div.flash_div = 2;      // flash  = MCG/3
        else if(clk <= 4 * MAX_FLASH_CLK)   mcg_div.flash_div = 3;      // flash  = MCG/4
        else if(clk <= 5 * MAX_FLASH_CLK)   mcg_div.flash_div = 4;      // flash  = MCG/5
        else if(clk <= 6 * MAX_FLASH_CLK)   mcg_div.flash_div = 5;      // flash  = MCG/6
        else if(clk <= 7 * MAX_FLASH_CLK)   mcg_div.flash_div = 6;      // flash  = MCG/7
        else if(clk <= 8 * MAX_FLASH_CLK)   mcg_div.flash_div = 7;      // flash  = MCG/8
        else if(clk <= 9 * MAX_FLASH_CLK)   mcg_div.flash_div = 8;      // flash  = MCG/9
        else if(clk <= 10 * MAX_FLASH_CLK)   mcg_div.flash_div = 9;     // flash  = MCG/10
        else if(clk <= 11 * MAX_FLASH_CLK)   mcg_div.flash_div = 10;    // flash  = MCG/11
        else if(clk <= 12 * MAX_FLASH_CLK)   mcg_div.flash_div = 11;    // flash  = MCG/12
        else if(clk <= 13 * MAX_FLASH_CLK)   mcg_div.flash_div = 12;    // flash  = MCG/13
        else if(clk <= 14 * MAX_FLASH_CLK)   mcg_div.flash_div = 13;    // flash  = MCG/14
        else if(clk <= 15 * MAX_FLASH_CLK)   mcg_div.flash_div = 14;    // flash  = MCG/15
        else                                mcg_div.flash_div = 15;     // flash  = MCG/16
    }
}


/*!
 *  @brief      PLL超频
 *  @param      PLL_e       频率设置参数
 *  @return     超频频率（MHz）
 *  @since      v5.0
 *  @warning    此函数只能在 复位后没进行任何频率设置情况下调用，即MCG在FEI模式下才可调用
 *  Sample usage:       uint8 clk = pll_init(PLL100);        //超频
 */
uint16 pll_init(PLL_e pll)
{

    mcg_div_count( pll);

    set_sys_dividers(mcg_div.core_div, mcg_div.bus_div, mcg_div.flex_div, mcg_div.flash_div);

    SIM_SOPT1 = ((SIM_SOPT1) & (uint32_t)(~(SIM_SOPT1_OSC32KSEL_MASK))) | ((SIM_SOPT1_OSC32KSEL(2)) & (SIM_SOPT1_OSC32KSEL_MASK));
    SIM_SOPT2 = ((SIM_SOPT2) & (uint32_t)(~(SIM_SOPT2_PLLFLLSEL_MASK))) | (SIM_SOPT2_PLLFLLSEL(1));
    SIM_SOPT2 = ((SIM_SOPT2) & (uint32_t)(~(SIM_SOPT2_TPMSRC_MASK))) | (SIM_SOPT2_TPMSRC(1)) ;

    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

    PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));


  MCG_SC = 0;
  MCG_C2 = (MCG_C2 & (uint8_t)(~(MCG_C2_RANGE_MASK)))
            | (MCG_C2_RANGE(2) | MCG_C2_EREFS_MASK) ;
  MCG_C2 &= ~MCG_C2_LP_MASK;
  OSC_CR = OSC_CR_ERCLKEN_MASK;

    //上电复位后，单片机会自动进入 FEI 模式，使用 内部参考时钟

    MCG_C11 &= ~MCG_C11_PLLCS_MASK;

    //FEI -> FBE
    MCG_C6 &= ~MCG_C6_PLLS_MASK;


    MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(7) | MCG_C1_IRCLKEN_MASK;


    while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) {
    }
    while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) {
    }


    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2) {}; //等待选择外部参考时钟

 MCG_C4 = 0;


 //现在已经进入了 FBE模式
    MCG_C5 = MCG_C5_PLLSTEN_MASK | MCG_C5_PRDIV(mcg_cfg[pll].prdiv);
    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(mcg_cfg[pll].vdiv) ;

    while (!(MCG_S & MCG_S_PLLST_MASK)) {};

    while (!(MCG_S & MCG_S_LOCK0_MASK)) {};

    // 现在已经进入了 PBE 模式

    // PBE -> PEE
    MCG_C1 &= ~MCG_C1_CLKS_MASK;

    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3) {};//等待选择输出PLL

    // 现在已经进入了 PEE 模式

    return mcg_cfg[pll].clk;
} //pll_init


/*!
 *  @brief      时钟分频设置函数
 *  @param      outdiv1    内核分频系数，       core    clk = MCG / (outdiv1 +1)
 *  @param      outdiv2    bus分频系数，        bus     clk = MCG / (outdiv2 +1)
 *  @param      outdiv3    flexbus分频系数，    flexbus clk = MCG / (outdiv3 +1)
 *  @param      outdiv4    flash分频系数，      flash   clk = MCG / (outdiv4 +1)
 *  @since      v1.0
 *  @author     飞思卡尔公司
 *  Sample usage:       set_sys_dividers(0,1, 9,3);     // core clk = MCG ; bus clk = MCG / 2 ; flexbus clk = MCG /10 ; flash clk = MCG / 4;
 */
__RAMFUNC  void set_sys_dividers(uint32 outdiv1, uint32 outdiv2, uint32 outdiv3, uint32 outdiv4)
{
    /*
    * This routine must be placed in RAM. It is a workaround for errata e2448.
    * Flash prefetch must be disabled when the flash clock divider is changed.
    * This cannot be performed while executing out of flash.
    * There must be a short delay after the clock dividers are changed before prefetch
    * can be re-enabled.
    */
    uint32 temp_reg;
    uint8 i;

    temp_reg = FMC_PFAPR; // store present value of FMC_PFAPR

    // set M0PFD through M7PFD to 1 to disable prefetch
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                 | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                 | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK;

    // set clock dividers to desired value
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(outdiv1) | SIM_CLKDIV1_OUTDIV2(outdiv2)
                  | SIM_CLKDIV1_OUTDIV3(outdiv3) | SIM_CLKDIV1_OUTDIV4(outdiv4);

    // wait for dividers to change
    for (i = 0 ; i < outdiv4 ; i++)
        {}

    FMC_PFAPR = temp_reg; // re-store original value of FMC_PFAPR

    return;
}