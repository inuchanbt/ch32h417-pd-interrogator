/********************************** (C) COPYRIGHT  *******************************
 * File Name          : PD_Prot.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __PD_PROT_H__
#define __PD_PROT_H__

/*******************************************************************************/

#include "debug.h"
#include "pd_phy_h417.h"
#include "pd_msgtypes.h"

/* Connection status change */
void pDevice_Attached(void);		//Equipment connection
void pDevice_Unattached(void);		//Device removal

/* Key message */
void pProt_IDLE(void);				//Message reception without state machine

/* Source power negotiation process */
void pProt_TX_SrcCap(void);
void pProt_Wait_Request(void);
void pProt_RX_Request(void);
void pProt_TX_Accept(void);
void pProt_Set_Volt_Change(void);
void pProt_Volt_Change(void);
void pProt_TX_PS_RDY(void);

/* Sink power negotiation process */
void pProt_Wait_SrcCap(void);
void pProt_RX_SrcCap(void);
void pProt_TX_Request(void);
void pProt_RX_Accept(void);
void pProt_RX_PS_RDY(void);

void pProt_TX_SinkCap(void);
void pProt_RX_DRSwap(void);
void pProt_TX_SoftRst(void);
void pProt_SoftRst_RX_Accept(void);
void pProt_RX_SoftRst(void);
void pProt_Excute_SoftRst(void);
void pProt_RX_ChunkedMsg(void);

/* EPR Mode Probe — called from PD_VDM.c after Discover SVIDs completes or times out */
void PD_EPR_Enter_Probe_If_Capable(void);

/* Machine-readable analyzer result stream (@PD1 records). */
enum {
	PD_RESULT_DISC_DONE = 1,
	PD_RESULT_DISC_NO_RESPONSE,
	PD_RESULT_DISC_TX_FAILED,
	PD_RESULT_DISC_NAK,
	PD_RESULT_DISC_BUSY,
	PD_RESULT_DISC_REJECTED,
	PD_RESULT_DISC_INVALID
};

void PD_Result_OnAttach(void);
void PD_Result_SetCC(u8 cc);
void PD_Result_OnDetach(void);
void PD_Result_Poll(void);
void PD_Result_SetSourceIdentity(u16 vid, u16 pid, u32 id_header, u32 product_vdo);
void PD_Result_SetCableIdentity(u16 vid, u16 pid, u8 product_type,
		u8 current_code, u8 max_voltage_code, u8 usb_speed,
		u32 id_header, u32 product_vdo, u32 cable_vdo);
void PD_Result_SetSourceDiscovery(u8 result);

typedef struct {
	union {
		u32 Data;
		struct {
			u32 MaxCurrent:10;
			u32 Voltage:10;
			u32 PeakCurrent:2;
			u32 Reserved:1;
			u32 EPRMode:1;
			u32 UnchunkedExtended:1;
			u32 DualRoleData:1;
			u32 USBComm:1;
			u32 UnconstrainedPower:1;
			u32 USBSuspend:1;
			u32 DualRolePower:1;
			u32 FixedSupply:2;
		};
	};
} st_SrcCap_Fixed;


#endif
