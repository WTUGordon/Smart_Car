/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_ftm.c
 * @brief      FTM��ʱ��������
 * @author     ɽ��Ƽ�
 * @version    v5.1
 * @date       2014-04-25
 */


/*
 * ����ͷ�ļ�
 */
#include "common.h"
#include  "MK66_FTM.h"

/*
 * ��������
 */
FTM_MemMapPtr FTMN[4] = FTM_BASE_PTRS; //�����ĸ�ָ�����鱣�� FTMn_e �ĵ�ַ

/*!
 *  @brief      ��ʼ��FTM �Ķ˿�
 *  @param      FTMn_e    ģ��ţ�FTM0��  FTM1��  FTM2��
 *  @param      FTM_CHn_e     ͨ���ţ�CH0~CH7��
 *  @since      v5.0
 */
static void FTM_port_mux(FTMn_e ftmn, FTM_CHn_e ch)
{
    /******************* ����ʱ�� �� ����IO��*******************/
    switch(ftmn)
    {
    case FTM0:
        SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;       //ʹ��FTM0ʱ��
        switch(ch)
        {
        case FTM_CH0:
            if(FTM0_CH0 == PTC1)
            {
                port_init(FTM0_CH0, ALT4);
            }
            else if(FTM0_CH0 == PTA3)
            {
                port_init(FTM0_CH0, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH1:
            if(FTM0_CH1 == PTC2)
            {
                port_init(FTM0_CH1, ALT4);
            }
            else if(FTM0_CH1 == PTA4)
            {
                port_init(FTM0_CH1, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH2:
            if(FTM0_CH2 == PTC3)
            {
                port_init(FTM0_CH2, ALT4);
            }
            else if(FTM0_CH2 == PTA5)
            {
                port_init(FTM0_CH2, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH3:
            if(FTM0_CH3 == PTC4)
            {
                port_init(FTM0_CH3, ALT4);
            }
            else if(FTM0_CH3 == PTA6)
            {
                port_init(FTM0_CH3, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH4:
            if(FTM0_CH4 == PTD4)
            {
                port_init(FTM0_CH4, ALT4);
            }
            else if(FTM0_CH4 == PTA7)
            {
                port_init(FTM0_CH4, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH5:
            if(FTM0_CH5 == PTD5)
            {
                port_init(FTM0_CH5, ALT4);
            }
            else if(FTM0_CH5 == PTA0)
            {
                port_init(FTM0_CH5, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH6:
            if(FTM0_CH6 == PTD6)
            {
                port_init(FTM0_CH6, ALT4);
            }
            else if(FTM0_CH6 == PTA1)
            {
                port_init(FTM0_CH6, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH7:
            if(FTM0_CH7 == PTD7)
            {
                port_init(FTM0_CH7, ALT4);
            }
            else if(FTM0_CH7 == PTA2)
            {
                port_init(FTM0_CH7, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;
        default:
            return;
        }
        break;

    case FTM1:
        SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;       //ʹ��FTM1ʱ��
        switch(ch)
        {
        case FTM_CH0:
            if((FTM1_CH0 == PTA8) || (FTM1_CH0 == PTA12) || (FTM1_CH0 == PTB0) )
            {
                port_init(FTM1_CH0, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;


        case FTM_CH1:
            if((FTM1_CH1 == PTA9) || (FTM1_CH1 == PTA13) || (FTM1_CH1 == PTB1) )
            {
                port_init(FTM1_CH1, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        default:
            return;
        }
        break;

    case FTM2:
        SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;                           //ʹ��FTM2ʱ��
        switch(ch)
        {
        case FTM_CH0:
            if((FTM2_CH0 == PTA10) || (FTM2_CH0 == PTB18) )
            {
                port_init(FTM2_CH0, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        case FTM_CH1:
            if((FTM2_CH1 == PTA11) || (FTM2_CH1 == PTB19))
            {
                port_init(FTM2_CH1, ALT3);
            }
            else
            {
                ASSERT(0);                      //���ùܽ�����
            }
            break;

        default:
            return;
        }
        break;
    default:
        break;
    }
}

/*!
 *  @brief      ��ʼ��FTM ��PWM ����
 *  @param      FTMn_e    ģ��ţ�FTM0��  FTM1��  FTM2��
 *  @param      FTM_CHn_e     ͨ���ţ�CH0~CH7��
 *  @param      freq    Ƶ�ʣ���λΪHz��
 *  @param      duty    ռ�ձȷ��ӣ�ռ�ձ� = duty / FTMn_PRECISON
 *  @since      v5.0
 *  @note       ͬһ��FTM��PWMƵ���Ǳ���һ���ģ���ռ�ձȿɲ�һ������3��FTM�����������3����ͬƵ��PWM
 *  Sample usage:       FTM_PWM_init(FTM0, FTM_CH6,200, 10);    //��ʼ�� FTM0_CH6 Ϊ Ƶ�� 200Hz ��PWM��ռ�ձ�Ϊ 10/FTM0_PRECISON
 */
void FTM_PWM_init(FTMn_e ftmn, FTM_CHn_e ch, uint32 freq, uint32 duty)
{
    uint32 clk_hz ;
    uint16 mod;
    uint8  ps;
    uint16 cv;

    ASSERT( (ftmn == FTM0) || ( (ftmn == FTM1 || ftmn == FTM2 ) && (ch <= FTM_CH1))   );  //��鴫�ݽ�����ͨ���Ƿ���ȷ

    /******************* ����ʱ�� �� ����IO��*******************/
    FTM_port_mux(ftmn,ch);

    /*       ����Ƶ������        */
    //  �� CPWMS = 1 ����˫�߲�׽���壬�� PMWƵ�� =  busƵ�� /2 /(2^Ԥ��Ƶ����)/ģ��
    //  �� CPWMS = 0 �������߲�׽���壬�� PMWƵ�� =  busƵ��    /(2^Ԥ��Ƶ����)/ģ��
    //  EPWM������ ��MOD - CNTIN + 0x0001   (CNTIN ��Ϊ0)
    //  �����ȣ�CnV - CNTIN

    //  ģ�� MOD < 0x10000
    //  Ԥ��Ƶ���� PS  < 0x07
    //  Ԥ��Ƶ���� PS ԽСʱ��ģ�� mod ��Խ�󣬼�����Խ��׼��PWM�����Ϊ׼ȷ
    //  MOD  = clk_hz/(freq*(1 << PS)) < 0x10000  ==>  clk_hz/(freq*0x10000) < (1<< PS)  ==>  (clk_hz/(freq*0x10000) >> PS) < 1
    //  �� (((clk_hz/0x10000 )/ freq ) >> PS ) < 1

    // �� CPWMS = 0 �������߲�׽����Ϊ��
    clk_hz = (bus_clk_khz * 1000) ;     // busƵ��

    mod = (clk_hz >> 16 ) / freq ;      // ��ʱ�� mod ����һ��
    ps = 0;
    while((mod >> ps) >= 1)             // �� (mod >> ps) < 1 ���˳� while ѭ�� ������ PS ����Сֵ
    {
        ps++;
    }

    ASSERT(ps <= 0x07);                 // ���ԣ� PS ���Ϊ 0x07 ��������ֵ���� PWMƵ�����ù��ͣ��� Bus Ƶ�ʹ���

    mod = (clk_hz >> ps) / freq;        // �� MOD ��ֵ

    switch(ftmn)                        // ��ֵ CNTIN ��Ϊ0 �������ȣ�CnV - CNTIN ���� CnV ���� �������ˡ�
    {
        // EPWM������ �� MOD - CNTIN + 0x0001 == MOD - 0 + 1
        // �� CnV = (MOD - 0 + 1) * ռ�ձ� = (MOD - 0 + 1) * duty/ FTM_PRECISON
    case FTM0:
        cv = (duty * (mod - 0 + 1)) / FTM0_PRECISON;
        break;

    case FTM1:
        cv = (duty * (mod - 0 + 1)) / FTM1_PRECISON;
        break;

    case FTM2:
        cv = (duty * (mod - 0 + 1)) / FTM2_PRECISON;
        break;

    default:
        break;
    }

    /******************** ѡ�����ģʽΪ ���ض���PWM *******************/
    //ͨ��״̬���ƣ�����ģʽ��ѡ�� ���ػ��ƽ
    FTM_CnSC_REG(FTMN[ftmn], ch) &= ~FTM_CnSC_ELSA_MASK;
    FTM_CnSC_REG(FTMN[ftmn], ch)  = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
    // MSnB:MSnA = 1x       ���ض���PWM
    // ELSnB:ELSnA = 10     �ȸߺ��
    // ELSnB:ELSnA = 11     �ȵͺ��

    /******************** ����ʱ�Ӻͷ�Ƶ ********************/
    FTM_SC_REG(FTMN[ftmn])    = ( 0
                                  //| FTM_SC_CPWMS_MASK         //0�������ؼ���ģʽ ��1�� �����ؼ���ģʽѡ�� ��ע���˱�ʾ 0��
                                  | FTM_SC_PS(ps)             //��Ƶ���ӣ���Ƶϵ�� = 2^PS
                                  | FTM_SC_CLKS(1)            //ʱ��ѡ�� 0��ûѡ��ʱ�ӣ����ã� 1��bus ʱ�ӣ� 2��MCGFFCLK�� 3��EXTCLK�� ��SIM_SOPT4 ѡ������ܽ� FTM_CLKINx��
                                  //| FTM_SC_TOIE_MASK        //����ж�ʹ�ܣ�ע���˱�ʾ ��ֹ����жϣ�
                                );
    FTM_MOD_REG(FTMN[ftmn])   = mod;                        //ģ��, EPWM������Ϊ ��MOD - CNTIN + 0x0001
    FTM_CNTIN_REG(FTMN[ftmn]) = 0;                          //��������ʼ��ֵ�����������ȣ�(CnV - CNTIN).
    FTM_CnV_REG(FTMN[ftmn], ch) = cv;
    FTM_CNT_REG(FTMN[ftmn])   = 0;                          //��������ֻ�е�16λ���ã�д�κ�ֵ���˼Ĵ������������ CNTIN ��ֵ��
}

/*!
 *  @brief      ����FTM ��PWM ͨ��ռ�ձ�
 *  @param      FTMn_e    ģ��ţ�FTM0��  FTM1��  FTM2��
 *  @param      FTM_CHn_e     ͨ���ţ�CH0~CH7��
 *  @param      duty    ռ�ձȷ��ӣ�ռ�ձ� = duty / FTMn_PRECISON
 *  @since      v5.0
 *  @note       ͬһ��FTM��PWMƵ���Ǳ���һ���ģ���ռ�ձȿɲ�һ������3��FTM�����������3����ͬƵ��PWM
 *  Sample usage:       FTM_PWM_Duty(FTM0, FTM_CH6, 10);    //���� FTM0_CH6ռ�ձ�Ϊ 10/FTM0_PRECISON
 */
void FTM_PWM_Duty(FTMn_e ftmn, FTM_CHn_e ch, uint32 duty)
{
    uint32 cv;
    uint32 mod = 0;

    ASSERT( (ftmn == FTM0) || ( (ftmn == FTM1 || ftmn == FTM2 ) && (ch <= FTM_CH1)) ); //��鴫�ݽ�����ͨ���Ƿ���ȷ

    switch(ftmn)
    {
    case FTM0:
        ASSERT(duty <= FTM0_PRECISON);     //�ö��Լ�� ռ�ձ��Ƿ����
        break;

    case FTM1:
        ASSERT(duty <= FTM1_PRECISON);     //�ö��Լ�� ռ�ձ��Ƿ����
        break;

    case FTM2:
        ASSERT(duty <= FTM2_PRECISON);     //�ö��Լ�� ռ�ձ��Ƿ����
        break;

    default:
        break;
    }


    //ռ�ձ� = (CnV-CNTIN)/(MOD-CNTIN+1)

    do
    {
        mod = FTM_MOD_REG(FTMN[ftmn]);        //��ȡ MOD ��ֵ
    }
    while(mod == 0);      //��һ�Σ�������0 ����Ҫ���ȡ���Ρ�

    switch(ftmn)
    {
    case FTM0:
        cv = (duty * (mod - 0 + 1)) / FTM0_PRECISON;
        break;

    case FTM1:
        cv = (duty * (mod - 0 + 1)) / FTM1_PRECISON;
        break;

    case FTM2:
        cv = (duty * (mod - 0 + 1)) / FTM2_PRECISON;
        break;
    default:
        break;
    }

    // ����FTMͨ��ֵ
    FTM_CnV_REG(FTMN[ftmn], ch) = cv;

}

/*!
 *  @brief      ����FTM��Ƶ��
 *  @param      freq    Ƶ�ʣ���λΪHz��
 *  @since      v5.0
 *  @note       �޸�PWMƵ�ʺ󣬱������ FTM_PWM_Duty ��������ռ�ձȡ�ͬһ��ģ�飬PWMƵ�ʱ�����ͬ��
 *  Sample usage:       FTM_PWM_freq(FTM0,200);    //���� FTM0 �� Ƶ�� Ϊ 200Hz
 */
void FTM_PWM_freq(FTMn_e ftmn, uint32 freq)             //����FTM��Ƶ��
{
    uint32 clk_hz = (bus_clk_khz * 1000) >> 1;        //busƵ��/2
    uint32 mod;
    uint8 ps;

    /*       ����Ƶ������        */
    // �� CPWMS = 0 �������߲�׽����Ϊ��
    clk_hz = (bus_clk_khz * 1000) ;     // busƵ��

    mod = (clk_hz >> 16 ) / freq ;      // ��ʱ�� mod ����һ��
    ps = 0;
    while((mod >> ps) >= 1)             // �� (mod >> ps) < 1 ���˳� while ѭ�� ������ PS ����Сֵ
    {
        ps++;
    }

    ASSERT(ps <= 0x07);                 // ���ԣ� PS ���Ϊ 0x07 ��������ֵ���� PWMƵ�����ù��ͣ��� Bus Ƶ�ʹ���

    mod = (clk_hz >> ps) / freq;        // �� MOD ��ֵ

    /******************** ����ʱ�Ӻͷ�Ƶ ********************/
    FTM_SC_REG(FTMN[ftmn])    = ( 0
                                  //| FTM_SC_CPWMS_MASK         //0�������ؼ���ģʽ ��1�� �����ؼ���ģʽѡ�� ��ע���˱�ʾ 0��
                                  | FTM_SC_PS(ps)             //��Ƶ���ӣ���Ƶϵ�� = 2^PS
                                  | FTM_SC_CLKS(1)            //ʱ��ѡ�� 0��ûѡ��ʱ�ӣ����ã� 1��bus ʱ�ӣ� 2��MCGFFCLK�� 3��EXTCLK�� ��SIM_SOPT4 ѡ������ܽ� FTM_CLKINx��
                                  //| FTM_SC_TOIE_MASK        //����ж�ʹ�ܣ�ע���˱�ʾ ��ֹ����жϣ�
                                );
    FTM_CNTIN_REG(FTMN[ftmn]) = 0;      //��������ʼ��ֵ�����������ȣ�(CnV - CNTIN).
    FTM_MOD_REG(FTMN[ftmn])   = mod;    //ģ��, EPWM������Ϊ ��MOD - CNTIN + 0x0001
    FTM_CNT_REG(FTMN[ftmn])   = 0;      //��������ֻ�е�16λ���ã�д�κ�ֵ���˼Ĵ������������ CNTIN ��ֵ��
}

//////////////////////////////// ����Ϊ���PWM  //////////////////////////////////////////

//////////////////////////////// ����Ϊ���벶׽ //////////////////////////////////////////

/*************************************************************************
*                             ɽ��Ƕ��ʽ����������
*
*  �������ƣ�FTM_Input_init
*  ����˵�������벶׽��ʼ������
*  ����˵����FTMn        ģ��ţ�FTM0��  FTM1��  FTM2��
*            FTM_CHn_e         ͨ���ţ�CH0~CH7��
*            Input_cfg   ���벶׽���ã�Rising��Falling��Rising_or_Falling�������ز�׽���½��ز�׽�������ز�׽
*  �������أ���
*  �޸�ʱ�䣺2012-1-26
*  ��    ע��CH0~CH3����ʹ�ù�������δ����⹦��
*************************************************************************/
void FTM_Input_init(FTMn_e ftmn, FTM_CHn_e ch, FTM_Input_cfg cfg,FTM_PS_e ps)
{
    ASSERT( (ftmn == FTM0) || ( (ftmn == FTM1 || ftmn == FTM2 ) && (ch <= FTM_CH1)) ); //��鴫�ݽ�����ͨ���Ƿ���ȷ

    /******************* ����ʱ�� �� ����IO��*******************/
    FTM_port_mux(ftmn,ch);

    /******************* ����Ϊ���벶׽���� *******************/
    switch(cfg)
    {
        //���벶׽ģʽ�£�DECAPEN = 0 �� DECAPEN = 0 ��CPWMS = 0�� MSnB:MSnA = 0

        // ELSnB:ELSnA         1          10          11
        // ����             ������      �½���      ������

    case FTM_Rising:    //�����ش���
        FTM_CnSC_REG(FTMN[ftmn], ch) |=  ( FTM_CnSC_ELSA_MASK  | FTM_CnSC_CHIE_MASK );                   //��1
        FTM_CnSC_REG(FTMN[ftmn], ch) &= ~( FTM_CnSC_ELSB_MASK  | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK); //��0
        break;

    case FTM_Falling:   //�½��ش���
        FTM_CnSC_REG(FTMN[ftmn], ch) |= (FTM_CnSC_ELSB_MASK  | FTM_CnSC_CHIE_MASK );                    //��1
        FTM_CnSC_REG(FTMN[ftmn], ch) &= ~( FTM_CnSC_ELSA_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK); //��0
        break;

    case FTM_Rising_or_Falling: //�����ء��½��ض�����
        FTM_CnSC_REG(FTMN[ftmn], ch) |=  ( FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK  | FTM_CnSC_CHIE_MASK ); //��1
        FTM_CnSC_REG(FTMN[ftmn], ch) &= ~( FTM_CnSC_MSB_MASK  | FTM_CnSC_MSA_MASK); //��0
        break;
    }

    FTM_SC_REG(FTMN[ftmn]) = ( 0
                              | FTM_SC_CLKS(0x1)       //ѡ�� bus ʱ��
							  | FTM_SC_PS(ps)          //ѡ�� ��Ƶϵ��
                             );
    FTM_MODE_REG(FTMN[ftmn])    |= FTM_MODE_WPDIS_MASK; //��ֹд����
    FTM_COMBINE_REG(FTMN[ftmn]) = 0;
    FTM_MODE_REG(FTMN[ftmn])    &= ~FTM_MODE_FTMEN_MASK;    //ʹ��FTM
    FTM_CNTIN_REG(FTMN[ftmn])   = 0;

    FTM_STATUS_REG(FTMN[ftmn])  = 0x00;               //���жϱ�־λ

    //�������벶׽�ж�
    //enable_irq(FTM0_IRQn + ftmn);
}

/*!
 *  @brief      FTM�����жϷ�����
 *  @since      v5.0
 *  @warning    �˺�����Ҫ�û������Լ�������ɣ�����������ṩһ��ģ��
 *  Sample usage:       set_vector_handler(FTM0_IRQn , FTM1_Input_test_handler);    //�� FTM1_Input_test_handler ������ӵ��ж�����������Ҫ�����ֶ�����
 */
void FTM1_Input_test_handler(void)
{
    uint8 s = FTM1_STATUS;             //��ȡ��׽�ͱȽ�״̬  All CHnF bits can be checked using only one read of STATUS.
    uint8 CHn;

    FTM1_STATUS = 0x00;             //���жϱ�־λ

    CHn = 0;
    if( s & (1 << CHn) )
    {
        //FTM_IRQ_DIS(FTM1, CHn);     //��ֹ���벶׽�ж�
        /*     �û�����       */

        /*********************/
        //FTM_IRQ_EN(FTM1, CHn); //�������벶׽�ж�

    }

    /* ������� n=1 ��ģ�棬����ģ������� */
    CHn = 1;
    if( s & (1 << CHn) )
    {
        //FTM_IRQ_EN(FTM1, CHn); //�������벶׽�ж�
        /*     �û�����       */


        /*********************/
        //�����������￪�����벶׽�ж�
        //FTM_IRQ_EN(FTM1, CHn); //�������벶׽�ж�
    }
}

//////////////////////////////// ����Ϊ���벶׽  //////////////////////////////////////////

//////////////////////////////// ����Ϊ�������� //////////////////////////////////////////

/*!
 *  @brief      ��ʼ��FTM ���������� ����
 *  @param      FTMn_e    ģ��ţ� FTM1��  FTM2��
 *  @since      v5.0
 *  Sample usage:       FTM_QUAD_Init(FTM1);    //��ʼ�� FTM1 Ϊ��������ģʽ
 */
void FTM_QUAD_Init(FTMn_e ftmn)
{
    ASSERT( (ftmn == FTM1) || (ftmn == FTM2 ) ); //��鴫�ݽ�����ͨ���Ƿ���ȷ

    /******************* ����ʱ�� �� ����IO��*******************/
    switch(ftmn)
    {

    case FTM1:
        SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;       //ʹ��FTM1ʱ��
                /*��Ϊ������������װ�����෴�����������ʼ����Ȼһ��Ϊ��һ��Ϊ���������ڳ�ʼFTM1ʱ��B�ࣨ���򣩵���*/
        FTM_QDCTRL_REG(FTMN[ftmn]) |=   (0
                                    | FTM_QDCTRL_PHBPOL_MASK
                                     );//���ú�С����ǰ�ٶ�Ϊ��������Ϊ��
        if(FTM1_QDPHA == PTA8)                  //�ܽŸ���
        {
            port_init(FTM1_QDPHA, ALT6);
        }
        else if(FTM1_QDPHA == PTA12)
        {
            port_init(FTM1_QDPHA, ALT7);
        }
        else if(FTM1_QDPHA == PTB0)
        {
            port_init(FTM1_QDPHA, ALT6);
        }
        else
        {
            ASSERT(0);                          //���ԣ����õĹܽŲ�����Ҫ��
        }

        if(FTM1_QDPHB == PTA9)
        {
            port_init(FTM1_QDPHB, ALT6);
        }
        else if(FTM1_QDPHB == PTA13)
        {
            port_init(FTM1_QDPHB, ALT7);
        }
        else if(FTM1_QDPHB == PTB1)
        {
            port_init(FTM1_QDPHB, ALT6);
        }
        else
        {
            ASSERT(0);                          //���ԣ����õĹܽŲ�����Ҫ��
        }
        break;

    case FTM2:
        SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;                           //ʹ��FTM2ʱ��
        if(FTM2_QDPHA == PTA10)                  //�ܽŸ���
        {
            port_init(FTM2_QDPHA, ALT6);
        }
        else if(FTM2_QDPHA == PTB18)
        {
            port_init(FTM2_QDPHA, ALT6);
//            PORTB_PCR18= PORT_PCR_MUX(6);
        }
        else
        {
            ASSERT(0);                          //���ԣ����õĹܽŲ�����Ҫ��
        }

        if(FTM2_QDPHB == PTA11)                  //�ܽŸ���
        {
            port_init(FTM2_QDPHB, ALT6);
        }
        else if(FTM2_QDPHB == PTB19)
        {
            port_init(FTM2_QDPHB, ALT6);
//            PORTB_PCR19= PORT_PCR_MUX(6);
        }
        else
        {
            ASSERT(0);                          //���ԣ����õĹܽŲ�����Ҫ��
        }
        break;
    default:
        ASSERT(0);                              //���ԣ����õ�ģ������
        break;
    }

    FTM_MODE_REG(FTMN[ftmn])  |=    (0
                                     | FTM_MODE_WPDIS_MASK  //д������ֹ
                                     //| FTM_MODE_FTMEN_MASK   //ʹ�� FTM
                                    );
    FTM_QDCTRL_REG(FTMN[ftmn]) |=   (0
                                    | FTM_QDCTRL_QUADMODE_MASK
                                     );
    FTM_CNTIN_REG(FTMN[ftmn])   = 0;
    FTM_MOD_REG(FTMN[ftmn])     = FTM_MOD_MOD_MASK;
    FTM_QDCTRL_REG(FTMN[ftmn]) |=   (0
                                    | FTM_QDCTRL_QUADEN_MASK
                                     );
    FTM_MODE_REG(FTMN[ftmn])  |= FTM_QDCTRL_QUADEN_MASK;
    FTM_CNT_REG(FTMN[ftmn])     = 0;                    //��������ֻ�е�16λ���ã�д�κ�ֵ���˼Ĵ������������ CNTIN ��ֵ��
}

/*!
 *  @brief      ��ȡFTM �������� ��������
 *  @param      FTMn_e    ģ��ţ� FTM1��  FTM2��
 *  @since      v5.0
 *  Sample usage:       int16 count = FTM_QUAD_get(FTM1);    //��ȡ  FTM1 ������ ��������
 */
int16 FTM_QUAD_get(FTMn_e ftmn)
{
    int16 val;
    ASSERT( (ftmn == FTM1) || (ftmn == FTM2 ) );        //��鴫�ݽ�����ͨ���Ƿ���ȷ
    val = FTM_CNT_REG(FTMN[ftmn]);

    return val;
}

/*!
 *  @brief      �� FTM �������� ��������
 *  @param      FTMn_e    ģ��ţ� FTM1��  FTM2��
 *  @since      v5.0
 *  Sample usage:       FTM_QUAD_clean(FTM1);    //��λ FTM1 �������� ��������
 */
void FTM_QUAD_clean(FTMn_e ftmn)
{
    ASSERT( (ftmn == FTM1) || (ftmn == FTM2 ) ); //��鴫�ݽ�����ͨ���Ƿ���ȷ

    FTM_CNT_REG(FTMN[ftmn])     = 0;             //��������ֻ�е�16λ���ã�д�κ�ֵ���˼Ĵ������������ CNTIN ��ֵ��
}

//-------------------------------------------------------------------------*
//������: FTM_AB_get
//��  ��: ��ȡFTM���������������
//��  ��: ftmn:ģ����FTM1��FTM2
//��  ��: val
//��  ��: speed=FTM_AB_get(FTM2);
//-------------------------------------------------------------------------*
int16 FTM_AB_Get(FTMn_e ftmn)
{
    int16 val;

    ASSERT( (ftmn == FTM1) || (ftmn == FTM2 ) );        //��鴫�ݽ�����ͨ���Ƿ���ȷ

    val = FTM_CNT_REG(FTMN[ftmn]);

    FTM_CNT_REG(FTMN[ftmn]) = 0;             //��������

    return val;
}


