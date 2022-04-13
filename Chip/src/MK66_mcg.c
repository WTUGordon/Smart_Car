/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_mcg.c
 * @brief      MCG PLL����
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-29
 */


#include "common.h"
#include "MK66_mcg.h"


#if defined(MK66F18)

mcg_cfg_t mcg_cfg[PLL_MAX] =
{
    {0, 0, 0}, //PLLUSR ��ʹ��
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
 *  @brief      MCG ��Ƶ����
 *  @param      PLL_e       Ƶ�����ò���
 *  @since      v5.0
 */
void mcg_div_count(PLL_e pll)
{
    uint16 clk = mcg_cfg[pll].clk;
    if( (pll > PLLUSR) && (pll < PLL_MAX) )
    {
        //���÷�Ƶ
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
 *  @brief      PLL��Ƶ
 *  @param      PLL_e       Ƶ�����ò���
 *  @return     ��ƵƵ�ʣ�MHz��
 *  @since      v5.0
 *  @warning    �˺���ֻ���� ��λ��û�����κ�Ƶ����������µ��ã���MCG��FEIģʽ�²ſɵ���
 *  Sample usage:       uint8 clk = pll_init(PLL100);        //��Ƶ
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

    //�ϵ縴λ�󣬵�Ƭ�����Զ����� FEI ģʽ��ʹ�� �ڲ��ο�ʱ��

    MCG_C11 &= ~MCG_C11_PLLCS_MASK;

    //FEI -> FBE
    MCG_C6 &= ~MCG_C6_PLLS_MASK;


    MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(7) | MCG_C1_IRCLKEN_MASK;


    while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) {
    }
    while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) {
    }


    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2) {}; //�ȴ�ѡ���ⲿ�ο�ʱ��

 MCG_C4 = 0;


 //�����Ѿ������� FBEģʽ
    MCG_C5 = MCG_C5_PLLSTEN_MASK | MCG_C5_PRDIV(mcg_cfg[pll].prdiv);
    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(mcg_cfg[pll].vdiv) ;

    while (!(MCG_S & MCG_S_PLLST_MASK)) {};

    while (!(MCG_S & MCG_S_LOCK0_MASK)) {};

    // �����Ѿ������� PBE ģʽ

    // PBE -> PEE
    MCG_C1 &= ~MCG_C1_CLKS_MASK;

    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3) {};//�ȴ�ѡ�����PLL

    // �����Ѿ������� PEE ģʽ

    return mcg_cfg[pll].clk;
} //pll_init


/*!
 *  @brief      ʱ�ӷ�Ƶ���ú���
 *  @param      outdiv1    �ں˷�Ƶϵ����       core    clk = MCG / (outdiv1 +1)
 *  @param      outdiv2    bus��Ƶϵ����        bus     clk = MCG / (outdiv2 +1)
 *  @param      outdiv3    flexbus��Ƶϵ����    flexbus clk = MCG / (outdiv3 +1)
 *  @param      outdiv4    flash��Ƶϵ����      flash   clk = MCG / (outdiv4 +1)
 *  @since      v1.0
 *  @author     ��˼������˾
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