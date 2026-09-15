/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2025/03/01
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *
 * PD SNK Sample code
 *
 * This sample code may have compatibility issues and is for learning purposes only.
 
 * Make sure that the board is not powered on before use.
 * Be sure to pay attention to the voltage when changing the request
 * to prevent burning the board.
 *
 * There is an external 5.1K pull-down on CC1/CC2 (R5/R6 on nanoCH32H417).
 * The firmware keeps the internal PHY pull-down off by default so Rd is not
 * doubled. Define PD_SNK_USE_INTERNAL_RD=1 only on boards without external Rd.
 * CC_NO_PU keeps Source Rp off.
 *
 * Attach sequence: CC-only sink wait -> RX on attach -> Get_Src_Cap after
 * VBUS valid (if ADC enabled) or 200 ms timeout. Unsolicited SrcCap skips TX.
 *
 * Modify "PDO_Request( PDO_INDEX_1 )" in pd process.c,to modify the request voltage.
 *
 * According to the usage scenario of PD SNK, whether
 * it is removed or not should be determined by detecting
 * the Vbus voltage, this code only shows the detection
 * and the subsequent communication flow.
 */

#include "debug.h"
#include "hardware.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	SystemAndCoreClockUpdate();
	Delay_Init();

	USART_Printf_Init(460800);
	printf("V5F SystemCoreClk:%d\r\n", SystemCoreClock);


#if (Run_Core == Run_Core_V3FandV5F)
	HSEM_FastTake(HSEM_ID0);
	HSEM_ReleaseOneSem(HSEM_ID0, 0);

#elif (Run_Core == Run_Core_V3F)

#elif (Run_Core == Run_Core_V5F)
	Hardware();
#endif


	while(1)
	{

	}
}
