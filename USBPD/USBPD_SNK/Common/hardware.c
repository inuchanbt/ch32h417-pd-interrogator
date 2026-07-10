/********************************** (C) COPYRIGHT  *******************************
* File Name          : hardware.c
* Author             : WCH
* Version            : V1.0.2
* Date               : 2026/03/25
* Description        : This file provides all the hardware firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "hardware.h"
#include "PD_Process.h"
#include "pd_phy_h417.h"
#include "PD_Prot.h"
#include "PD_User.h"

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

volatile UINT8  Tim_Ms_Cnt = 0x00;

#if PD_VBUS_USE_ADC
static volatile UINT16 s_vbus_adc_raw = 0;
/* ~4.0 V on VBUS; tune for board divider (ADC ref = VDDIO). */
#define PD_VBUS_ADC_ON_MV  4000u
#endif

/*********************************************************************
 * @fn      PD_VBUS_Init
 *
 * @brief   Board VBUS sense init (optional ADC).
 *
 * @return  none
 */
void PD_VBUS_Init(void)
{
#if PD_VBUS_USE_ADC
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_ADC1 | RCC_HB2Periph_GPIOA, ENABLE);
    RCC_ADCCLKConfig(RCC_ADCCLKSource_USBHSPLL);
    RCC_ADCUSBHSPLLCLKAsSourceConfig(RCC_USBHS_Div36);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_LowPowerModeCmd(ADC1, DISABLE);
    ADC_SMP_ModeConfig(ADC1, ADC_Channel_0, ADC_SMP_CFG_MODE1);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
#endif
}

/*********************************************************************
 * @fn      PD_VBUS_Update
 *
 * @brief   Sample VBUS (when ADC enabled).
 *
 * @return  none
 */
void PD_VBUS_Update(void)
{
#if PD_VBUS_USE_ADC
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SMP_CFG_MODE1);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    s_vbus_adc_raw = ADC_GetConversionValue(ADC1);
#endif
}

/*********************************************************************
 * @fn      PD_VBUS_Valid
 *
 * @brief   True when VBUS is above ~4 V (ADC path only).
 *
 * @return  1: valid; 0: not valid / no sensor
 */
UINT8 PD_VBUS_Valid(void)
{
#if PD_VBUS_USE_ADC
    UINT32 mv;

    mv = (UINT32)s_vbus_adc_raw * 3300u / 4096u;
    /* Example: 100k/100k divider on VBUS -> double the ADC pin voltage. */
    mv *= 2u;
    return ( mv >= PD_VBUS_ADC_ON_MV ) ? 1 : 0;
#else
    return 0;
#endif
}

/*********************************************************************
 * @fn      TIM1_Init
 *
 * @brief   Initialize TIM1
 *
 * @return  none
 */
void TIM1_Init( u16 arr, u16 psc )
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
    RCC_HB2PeriphClockCmd( RCC_HB2Periph_TIM1, ENABLE );
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);
    TIM_ClearITPendingBit( TIM1, TIM_IT_Update );

	NVIC_SetPriority(TIM1_UP_IRQn, 3);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    TIM_ITConfig( TIM1, TIM_IT_Update, ENABLE );
    TIM_Cmd( TIM1, ENABLE );
}




/*********************************************************************
 * @fn      Hardware
 *
 * @brief   Hardware
 *
 * @return  none
 */
void Hardware(void)
{
#if PD_USE_ANALYZER
	printf( "PD Analyzer (H417)\r\n" );
    PD_VBUS_Init( );
    PD_Analyzer_Init( );
    Set_DevChk( DevRole_Sink );
    TIM1_Init( 999, 120-1);
	while(1)
	{
        TIM_ITConfig( TIM1, TIM_IT_Update , DISABLE );
        Tmr_Ms_Dlt = Tim_Ms_Cnt - Tmr_Ms_Cnt_Last;
        Tmr_Ms_Cnt_Last = Tim_Ms_Cnt;
        TIM_ITConfig( TIM1, TIM_IT_Update , ENABLE );
        PD_PHY_Poll( );
        PD_Ctl.Det_Timer += Tmr_Ms_Dlt;
        if ( PD_Ctl.Det_Timer > 1u )
        {
            PD_Ctl.Det_Timer = 0;
            PD_Analyzer_Det_Proc( );
        }
        PD_PHY_TickMs( Tmr_Ms_Dlt );
        PD_User_Timer( );
	}
#else
	printf( "PD SNK TEST\r\n" );
    PD_VBUS_Init( );
    PD_Init( );
    TIM1_Init( 999, 120-1);
	while(1)
	{
        /* Get the calculated timing interval value */
        TIM_ITConfig( TIM1, TIM_IT_Update , DISABLE );
        Tmr_Ms_Dlt = Tim_Ms_Cnt - Tmr_Ms_Cnt_Last;
        Tmr_Ms_Cnt_Last = Tim_Ms_Cnt;
        TIM_ITConfig( TIM1, TIM_IT_Update , ENABLE );
        PD_Ctl.Det_Timer += Tmr_Ms_Dlt;
        if( PD_Ctl.Det_Timer > 4 )
        {
            PD_Ctl.Det_Timer = 0;
            PD_Det_Proc( );
        }
        PD_Main_Proc( );
	}
#endif
}

#if Func_Run_V3F

/*********************************************************************
 * @fn      TIM1_UP_IRQHandler
 *
 * @brief   This function handles TIM1 interrupt.
 *
 * @return  none
 */
void TIM1_UP_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM1, TIM_IT_Update ) != RESET )
    {
        Tim_Ms_Cnt++;
        TIM_ClearITPendingBit( TIM1, TIM_IT_Update );
    }
}

#endif