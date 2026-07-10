/********************************** (C) COPYRIGHT *******************************
* File Name          : PD_User.C
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/10/27
* Description        : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
********************************************************************************/

#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "PD_User.H"
#include "PD_VDM.h"

//The device acts as a source, sending the Source_Capabilities message content
u32 SrcCap[] = 	{0x3A21912C,};	//5V3A
vu8 SrcCapCnt = 1;

//The device acts as a sink, sending the content of the Sink_Capabilities message.
/*
 * PDスニファーとして EPR_Mode Enter を成功させるため、Sink_Capabilities に
 * EPRCap=1 + EPR AVS Sink APDO を含める。
 * PD 3.1 spec 8.3.3.5.6: "A Sink shall not send an EPR_Mode Enter Data Message
 * unless it has sent Sink_Capabilities that include an EPR APDO."
 *
 * PDO[1]: Fixed 5V/3A — スニファーは実際にはこの契約で動作する
 *   bit[29]=DRP, bit[27]=HigherCap, bit[25]=DRD,
 *   bit[24]=UnchunkExt (EPR 拡張メッセージ対応を通知),
 *   bit[23]=EPRCap=1 (EPR エントリ条件)
 * PDO[2]: EPR AVS Sink APDO 15000-48000mV PDP=240W
 *   ソースへ EPR 全範囲対応を通知するための宣言。
 *   実際の高電圧 Request は送らず EPR Mode Exit で即時復帰する。
 */
u32 SinkCap[] = {
    0x2B8641F4,   /* Fixed 20V/5A: EPRCap=1(b23) UnchunkExt=1(b24) HigherCap=1(b27) DRP=1(b29) */
                  /* 20V を宣言することで EPR 対応ソースが EPR_Mode Enter を許可する             */
                  /* リクエストは常に SinkCap[0] 以下の最高 Fixed PDO を選択するため実害なし    */
    0xD3C096F0,   /* EPR AVS Sink APDO: 15000-48000mV PDP=240W (PD 3.2: bits[29:28]=01)          */
};
vu8 SinkCapCnt = 2;

/*********************************************************************
 * @fn      PD_User_Snk_DevIn
 *
 * @brief   The device is connected as a sink.
 *
 * @return  none
 */
void PD_User_Snk_DevIn(void)
{
	printf("CC attach (analyzer sink)\r\n");
}

/*********************************************************************
 * @fn      PD_User_Snk_Rx_SrcCap
 *
 * @brief   The device receives the Source_Capabilities message as a sink.
 *
 * @return  none
 */
void PD_User_Snk_Rx_SrcCap(void)
{

}

/*********************************************************************
 * @fn      PD_User_Snk_Rx_PS_RDY
 *
 * @brief   The device receives the PS_RDY message as a sink.
 *
 * @return  none
 */
void PD_User_Snk_Rx_PS_RDY(void)
{

}

/*********************************************************************
 * @fn      PD_User_Src_DevIn
 *
 * @brief   The device is connected as a source.
 *
 * @return  none
 */
void PD_User_Src_DevIn(void)
{
	printf("CC attach (analyzer src)\r\n");
}

/*********************************************************************
 * @fn      PD_User_Src_VoltChange
 *
 * @brief   The device is used as a source for voltage regulation.
 *
 * @return  none
 */

void PD_User_Src_VoltChange(void)
{
#if	( !DEF_EN_VOLTCHANGE )
	pProt_TX_PS_RDY();		//No pressure regulation
#else

	st_SrcCap_Fixed *tSrcCap = (st_SrcCap_Fixed *)&SrcCap[PD_PHY.savedRequest.ObjectPos-1];


	if ( PD_PHY.tSrcTransition ) {
		PD_PHY.tSrcTransition--;
		if ( !PD_PHY.tSrcTransition ) {

		}
	}
	else if ( PD_PHY.tPSTransition ) {

		float dat;
		PD_PHY.tPSTransition--;
		dat=GetADC_VBUS/4096*3.3/18*118;
		dat=dat*1000;
		if ( ( (dat > (tSrcCap->Voltage*50*0.95) ) && (dat < (tSrcCap->Voltage*50*1.05) ) ) || !PD_PHY.tPSTransition ) {
			pProt_TX_PS_RDY();
		}

	}
#endif
}

/*********************************************************************
 * @fn      PD_User_DevOut
 *
 * @brief   Device detection removal.
 *
 * @return  none
 */
void PD_User_DevOut(void)
{
	printf("CC disconnect\r\n");
}

/*********************************************************************
 * @fn      PD_User_DevOut
 *
 * @brief   User function, execute once every 1ms.
 *
 * @return  none
 */
void PD_User_Timer(void)
{
	PD_Cable_Sniff_Timer_1ms();
}
