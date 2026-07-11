/********************************** (C) COPYRIGHT  *******************************
 * File Name          : PD_VDM.c
 * Author             : WCH  (modified for PD sniffer)
 * Version            : V1.1.0
 * Date               : 2026/06/30
 * Description        : VDM handling + Discover Identity → SVIDs → Modes chain
 *
 * 変更点:
 *   - Discover Identity ACK の後に Discover SVIDs を自動送信
 *   - Discover SVIDs ACK を解析・表示し、発見した SVID ごとに Discover Modes を送信
 *   - Discover Modes ACK を解析・表示（Enter Mode は送信しない）
 *   - タイムアウト時は次の SVID へスキップまたは終了
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 *******************************************************************************/

#include <stdio.h>
#include "debug.h"
#include "PD_Prot.h"
#include "pd_phy_h417.h"
#include "pd_msgtypes.h"
#include "PD_VDM.h"
#include "string.h"
#include "ch211_stub.h"

st_VDM VDM_State = {0};
st_VDM_Header *rxVDM = (st_VDM_Header *)&PD_RX_BUF[1];
st_VDM_Header *txVDM = (st_VDM_Header *)&PD_TX_BUF[1];

/* ── Discover SVIDs/Modes で発見した SVID を保持する状態変数 ── */
#define MAX_DISC_SVIDS 8
static u16 VDM_Disc_SVIDs[MAX_DISC_SVIDS];
static u8  VDM_Disc_SVID_Count;
static u8  VDM_Disc_Mode_Index;

/* ── SOP' Cable (E-marker) プローブ状態 ── */
static u8  PD_Cable_Probe_Done     = 0;  /* 試行済み (pDevice_Unattached でリセット) */
static u8  s_cable_identity_logged = 0;  /* 受信済み Cable Identity 表示済み         */

/*
 * libCH32_USBPD ISR compares (USBPD STATUS & 3) to PD_PHY.RxSop:
 *   1 = SOP, 2 = SOP', 3 = SOP"  (not the 0x10/0x11 values in the header comment)
 */
#define SOP_RX_FILTER_SOP   1u
#define SOP_RX_FILTER_SOPP  2u
#define SOP_RX_SOP          SOP_RX_FILTER_SOP
#define SOP_RX_SOPP         SOP_RX_FILTER_SOPP
#define SOP_TX_SOPP         0x50u

static u16 s_sopp_sniff_ms         = 0;  /* legacy; cleared on reset */
static u16 s_sopp_sniff_delay_ms   = 0;

static u32 s_pd_ms_tick            = 0;
static u32 s_vbus_up_ms_tick       = 0;
static u8  s_cable_sniff_scheduled = 0;
static u8  s_vbus_seen_on          = 0;
static u16 s_source_probe_delay_ms = 0;
static u8  s_source_probe_done     = 0;

/*
 * SOP' packets are passively sniffed from BMC_AUX/LastRxSop in the PHY.
 * The sink must not source VCONN_Swap or SOP' Discover Identity; doing so can
 * steal the charger-to-cable exchange and make the source fall back to 3A.
 */
#define CABLE_SNIFF_START_MS    124u
#define CABLE_SNIFF_END_MS      133u

/* ── 前方宣言 ── */
static void VDM_Disc_SvidTimeout(void);
static void VDM_Disc_ModeTimeout(void);
static void VDM_Cable_Probe_End(void);
static void VDM_Print_Cable_Identity_ACK(const char *via);
static void VDM_Cable_Enable_Vconn(void);
static void VDM_Cable_Disable_Vconn(void);
static void VDM_Try_Passive_SOPP(void);
static void VDM_Source_Probe_Finish(const char *reason);
void pProt_TX_DISC_MODES_Next(void);

static u32 VDM_Read_DO(u8 index)
{
	u8 word = (u8)(1u + (index << 1));
	return (u32)PD_RX_BUF[word] | ((u32)PD_RX_BUF[word + 1u] << 16);
}

void PD_Source_VDM_Probe_Arm_Delayed(u16 ms)
{
	if ( s_source_probe_done ) return;
	if ( ms == 0u ) ms = 1u;
	if ( s_source_probe_delay_ms == 0u || ms < s_source_probe_delay_ms ) {
		s_source_probe_delay_ms = ms;
	}
}

void PD_Source_VDM_Probe_Cancel(void)
{
	s_source_probe_delay_ms = 0u;
}

void PD_Source_VDM_Probe_Reset(void)
{
	s_source_probe_delay_ms = 0u;
	s_source_probe_done = 0u;
}

void PD_Source_VDM_Probe_Tick(u8 delta_ms)
{
	if ( s_source_probe_delay_ms == 0u || s_source_probe_done ) return;
	if ( PD_DEVICE.ConnectStat == 0 ) {
		s_source_probe_delay_ms = 0u;
		return;
	}
	if ( s_source_probe_delay_ms > delta_ms ) {
		s_source_probe_delay_ms = (u16)(s_source_probe_delay_ms - delta_ms);
		return;
	}
	if ( VDM_State.Explicit_Contract_Established == 0 ||
	     PD_PHY.WaitMsgTx || PD_PHY.WaitMsgRx || (USBPD->CONTROL & PD_TX_EN) ) {
		s_source_probe_delay_ms = 1u;
		return;
	}

	s_source_probe_delay_ms = 0u;
	s_source_probe_done = 1u;
	VDM_Reset_Disc_Probe_Only();
	PD_PHY.TxSop = PD_PHY_TX_SOP;
	PD_PHY_Set_RxSop(PD_PHY_RX_SOP);
	PD_PHY_Set_ListenOnlySopp(0);
	printf("TX SOP Discover Identity (post power interrogation)\r\n");
	pProt_TX_DISC_IDENT();
	PD_PHY_FlushTxNow();
}

static void PD_Cable_Sniff_Stop(void)
{
	s_cable_sniff_scheduled = 0;
	s_sopp_sniff_ms         = 0;
	s_sopp_sniff_delay_ms   = 0;
	if ( PD_PHY.RxSop != SOP_RX_FILTER_SOP ) {
		PD_PHY_Set_RxSop(SOP_RX_FILTER_SOP);
	}
}

static void PD_Cable_Sniff_Vbus_Tick(void)
{
	u8 on = ( GetADC_VBUS >= vVBUSon ) ? 1u : 0u;

	if ( on ) {
		if ( !s_vbus_seen_on ) {
			s_vbus_seen_on    = 1;
			s_vbus_up_ms_tick = s_pd_ms_tick;
			if ( !s_cable_identity_logged ) {
				s_cable_sniff_scheduled = 1;
			}
		}
	} else {
		s_vbus_seen_on = 0;
	}
}

static void PD_Cable_Sniff_Process(void)
{
	if ( PD_DEVICE.ConnectStat == 0 ) {
		s_vbus_seen_on = 0;
	}

	/* Always SOP: RxSop=2 window caused orientation-dependent HRST (105/106/110/113-1). */
	s_cable_sniff_scheduled = 0;
	if ( PD_PHY.RxSop != SOP_RX_FILTER_SOP ) {
		PD_PHY_Set_RxSop(SOP_RX_FILTER_SOP);
	}
}

void PD_Cable_Sniff_Arm_Delayed(void)
{
	/* attach 通知: VBUS エッジより遅れても、ケーブル応答前ならスニフを継続 */
	if ( s_cable_identity_logged ) return;
	if ( s_vbus_seen_on && ( s_pd_ms_tick - s_vbus_up_ms_tick ) < CABLE_SNIFF_END_MS ) {
		s_cable_sniff_scheduled = 1;
	}
}

void PD_Cable_Sniff_Start_Window(u16 ms)
{
	(void)ms;
	/* 108: 固定ウィンドウ (CABLE_SNIFF_*_MS) を使用。手動呼び出しは Arm のみ。 */
	PD_Cable_Sniff_Arm_Delayed();
}

void PD_Cable_Sniff_Timer_1ms(void)
{
	s_pd_ms_tick++;
	PD_Cable_Sniff_Vbus_Tick();
	PD_Cable_Sniff_Process();
}

void PD_Cable_Sniff_On_SrcCap(void)
{
	PD_Cable_Sniff_Stop();
}

static void PD_Cable_Sniff_On_Message(void)
{
	PD_Cable_Sniff_Stop();
}

/* ── SVID の既知ベンダー名表示ヘルパー ── */
static void VDM_Print_SVID_Name(u16 svid)
{
	switch ( svid ) {
	case 0xFF01: printf(" (DisplayPort)");     break;
	case 0xFF00: printf(" (USB-IF Standard)"); break;
	case 0x8087: printf(" (Intel/TBT)");       break;
	case 0x05AC: printf(" (Apple)");           break;
	case 0x04B4: printf(" (Cypress)");         break;
	case 0x18D1: printf(" (Google)");          break;
	case 0x0B05: printf(" (ASUS)");            break;
	case 0x0955: printf(" (NVIDIA)");          break;
	case 0x1D6B: printf(" (USB-IF PD)");       break;
	default: break;
	}
}

/*
 * PD_RX_BUF レイアウト (u16 配列):
 *   [0]    = PD メッセージヘッダ
 *   [1][2] = VDM ヘッダ  (lo16, hi16)  … rxVDM->Command/SVID 等
 *   [3][4] = VDO[0]      (lo16, hi16)
 *   [5][6] = VDO[1]      (lo16, hi16)
 *   ...
 * 各 u32 VDO: high16=PD_RX_BUF[4+2i], low16=PD_RX_BUF[3+2i]
 *   (既存コードの id_header 抽出パターンに合わせる)
 */

/* ── Discover Identity ACK の解析・表示 ── */
static void VDM_Print_Discover_Identity_ACK(const char *role)
{
	u8 i;
	u8 bytes = 2 + rxHeader->NDO * 4;
	u8 words = 1 + rxHeader->NDO * 2;
	u8 *raw = (u8 *)PD_RX_BUF;
	u32 vdm_header = ( rxHeader->NDO >= 1 ) ? VDM_Read_DO(0) : 0u;
	u32 id_header  = ( rxHeader->NDO >= 2 ) ? VDM_Read_DO(1) : 0u;
	u32 cert_stat  = ( rxHeader->NDO >= 3 ) ? VDM_Read_DO(2) : 0u;
	u32 product    = ( rxHeader->NDO >= 4 ) ? VDM_Read_DO(3) : 0u;
	u16 vid = (u16)(id_header & 0xFFFF);
	u16 pid = (u16)((product >> 16) & 0xFFFF);
	u16 bcd = (u16)(product & 0xFFFF);
	u8 product_type_ufp = (u8)((id_header >> 27) & 0x07u);
	u8 product_type_dfp = (u8)((id_header >> 23) & 0x07u);
	u8 connector_type   = (u8)((id_header >> 21) & 0x03u);
	u8 modal_operation  = (u8)((id_header >> 26) & 0x01u);

	if ( words > 12 ) words = 12;
	if ( bytes > 44 ) bytes = 44;

	printf("\r\n");  /* PD_PHY_Header_Init が改行なしで "tx" を出力するため、ここで行を区切る */
	printf("RX SOP Discover Identity ACK / port partner (%s) NDO:%d\r\n", role, rxHeader->NDO);
	printf("RAW16:");
	for ( i = 0; i < words; i++ ) printf(" [%d]=%04X", i, PD_RX_BUF[i]);
	printf("\r\n");
	printf("RAW8:");
	for ( i = 0; i < bytes; i++ ) printf(" %02X", raw[i]);
	printf("\r\n");

	if ( rxHeader->NDO >= 1 ) printf("VDM Header:0x%08lX\r\n", (unsigned long)vdm_header);
	if ( rxHeader->NDO >= 2 ) {
		printf("ID Header VDO:0x%08lX VID:0x%04X UFPType:%u DFPType:%u Connector:%u Modal:%u\r\n",
		       (unsigned long)id_header, vid, (unsigned)product_type_ufp,
		       (unsigned)product_type_dfp, (unsigned)connector_type,
		       (unsigned)modal_operation);
	}
	if ( rxHeader->NDO >= 3 ) printf("Cert Stat/XID:0x%08lX\r\n", (unsigned long)cert_stat);
	if ( rxHeader->NDO >= 4 ) {
		printf("Product VDO:0x%08lX PID:0x%04X bcdDevice:0x%04X\r\n", (unsigned long)product, pid, bcd);
	}
	if ( rxHeader->NDO >= 5 ) {
		u32 extra = VDM_Read_DO(4);
		printf("Extra VDO[4]:0x%08lX\r\n", (unsigned long)extra);
	}
	if ( (vid == 0) && (pid == 0) ) {
		printf("Identity result: SOP ACK received, VID/PID are zero.\r\n");
	}
}

/* ===================================================================
 * SOP' Cable (E-marker) Identity decode & probe
 * =================================================================== */

static const char *VDM_Product_Type_Name(u8 t)
{
	static const char *names[] = {
		"Undefined", "PD Hub", "PD Peripheral", "Passive Cable",
		"Active Cable", "AMA", "Res6", "Res7"
	};
	return ( t < 8u ) ? names[t] : "Unknown";
}

static const char *VDM_Cable_Max_Voltage_Name(u8 v)
{
	switch ( v ) {
	case 0: return "20V";
	case 1: return "30V";
	case 2: return "40V";
	case 3: return "48V";
	default: return "?";
	}
}

static const char *VDM_USB_Highest_Speed_Name(u8 s)
{
	switch ( s ) {
	case 0: return "USB2 only";
	case 1: return "USB3.2 Gen1";
	case 2: return "USB4 Gen2";
	case 3: return "USB4 Gen3";
	case 4: return "USB4 Gen4";
	default: return "Unknown";
	}
}

static void VDM_Print_Cable_Identity_ACK(const char *via)
{
	u8  i;
	u8  bytes = 2 + rxHeader->NDO * 4;
	u8  words = 1 + rxHeader->NDO * 2;
	u8 *raw   = (u8 *)PD_RX_BUF;
	u32 vdm_header= ( rxHeader->NDO >= 1 ) ? VDM_Read_DO(0) : 0u;
	u32 id_header = ( rxHeader->NDO >= 2 ) ? VDM_Read_DO(1) : 0u;
	u32 cert_xid  = ( rxHeader->NDO >= 3 ) ? VDM_Read_DO(2) : 0u;
	u32 product   = ( rxHeader->NDO >= 4 ) ? VDM_Read_DO(3) : 0u;
	u32 cable_vdo = ( rxHeader->NDO >= 5 ) ? VDM_Read_DO(4) : 0u;
	u16 vid           = (u16)(id_header & 0xFFFFu);
	u16 pid           = (u16)((product >> 16) & 0xFFFFu);
	u16 bcd_device    = (u16)(product & 0xFFFFu);
	u8  product_type  = (u8)((id_header >> 27) & 0x07u);  /* Cable Product Type [29:27] */
	u8  connector_type= (u8)((id_header >> 21) & 0x03u);  /* ID Header VDO [22:21] */
	u8  curr_cap      = (u8)((cable_vdo >> 5) & 0x03u);
	u8  max_v         = (u8)((cable_vdo >> 9) & 0x03u);
	u8  usb_speed     = (u8)(cable_vdo & 0x07u);
	u8  vdo_version   = (u8)((cable_vdo >> 21) & 0x07u);
	u8  cable_latency = (u8)((cable_vdo >> 13) & 0x0Fu);
	u8  termination   = (u8)((cable_vdo >> 11) & 0x03u);

	if ( words > 12 ) words = 12;
	if ( bytes > 44 ) bytes = 44;

	printf("\r\n");
	printf("RX SOP' Discover Identity ACK / cable (%s) NDO:%d\r\n", via, rxHeader->NDO);
	printf("RAW16:");
	for ( i = 0; i < words; i++ ) printf(" [%d]=%04X", i, PD_RX_BUF[i]);
	printf("\r\n");
	printf("RAW8:");
	for ( i = 0; i < bytes; i++ ) printf(" %02X", raw[i]);
	printf("\r\n");
	if ( rxHeader->NDO >= 1 ) {
		printf("  VDM Header:0x%08lX\r\n", (unsigned long)vdm_header);
	}

	if ( rxHeader->NDO >= 2 ) {
		printf("  Cable ID Header: VID=0x%04X ProductType=%s ConnectorType=%u raw:0x%08lX\r\n",
		       vid, VDM_Product_Type_Name(product_type), (unsigned)connector_type,
		       (unsigned long)id_header);
	}
	if ( rxHeader->NDO >= 3 ) {
		printf("  Cert Stat/XID:0x%08lX\r\n", (unsigned long)cert_xid);
	}
	if ( rxHeader->NDO >= 4 ) {
		printf("  Product VDO:0x%08lX PID=0x%04X bcdDevice=0x%04X\r\n",
		       (unsigned long)product, pid, bcd_device);
	}

	if ( product_type == 3u && rxHeader->NDO >= 5 ) {
		printf("  Passive Cable VDO1:0x%08lX\r\n", (unsigned long)cable_vdo);
		printf("  Cable: type=Passive");
		if ( curr_cap == 1u )      printf(" current=3A");
		else if ( curr_cap == 2u ) printf(" current=5A");
		else                       printf(" current=DefaultUSB");
		printf(" maxV=%s", VDM_Cable_Max_Voltage_Name(max_v));
		printf(" VDOv=%u latency=%u termination=%u USB=%s\r\n",
		       (unsigned)vdo_version, (unsigned)cable_latency,
		       (unsigned)termination, VDM_USB_Highest_Speed_Name(usb_speed));
	}
	else if ( product_type == 4u && rxHeader->NDO >= 5 ) {
		printf("  Active Cable VDO1:0x%08lX\r\n", (unsigned long)cable_vdo);
		printf("  Cable: type=Active");
		if ( curr_cap == 1u )      printf(" current=3A");
		else if ( curr_cap == 2u ) printf(" current=5A");
		else                       printf(" current=DefaultUSB");
		printf(" maxV=%s VDOv=%u latency=%u termination=%u USB=%s\r\n",
		       VDM_Cable_Max_Voltage_Name(max_v), (unsigned)vdo_version,
		       (unsigned)cable_latency, (unsigned)termination,
		       VDM_USB_Highest_Speed_Name(usb_speed));
	}
	else if ( rxHeader->NDO >= 5 ) {
		printf("  Cable VDO1:0x%08lX (ProductType=%s)\r\n",
		       (unsigned long)cable_vdo, VDM_Product_Type_Name(product_type));
	}
	else {
		printf("  Cable Identity: incomplete response (need >=5 objects for Cable VDO1)\r\n");
	}
	if ( rxHeader->NDO >= 6 ) {
		printf("  Cable VDO2:0x%08lX\r\n", (unsigned long)VDM_Read_DO(5));
	}

	s_cable_identity_logged = 1;
	PD_Cable_Probe_Done     = 1;
	PD_Cable_Sniff_On_Message();
}

static void VDM_Cable_Enable_Vconn(void)
{
	u8 pin = CH211_I2C_ReadByte(PIN_STAT);
	/* 通信 CC の反対側に VCONN を供給 (CH211 CC1/CC2 VCE) */
	if ( pin & CCI1 ) {
		PD_Vconn_CC2;
	} else if ( pin & CCI2 ) {
		PD_Vconn_CC1;
	} else {
		PD_Vconn_CC1;  /* fallback */
	}
}

static void VDM_Cable_Disable_Vconn(void)
{
	PD_Float_CC1;
	PD_Float_CC2;
}

static void VDM_Cable_Probe_End(void)
{
	VDM_Cable_Disable_Vconn();
	PD_PHY.TxSop = 0x00u;
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	Delay_Ms(30);  /* SOP' 失敗後のバス落ち着き待ち (Sink_Cap/EPR 衝突防止) */
	PD_EPR_Enter_Probe_If_Capable();
}

static void __attribute__((unused)) VDM_Cable_Probe_Failed(void)
{
	PD_PHY.WaitMsgRx = 0;
	printf("Cable probe: aborted — continuing\r\n");
	VDM_Cable_Probe_End();
}

static void VDM_Cable_DiscIdent_RX(void)
{
	PD_PHY.WaitMsgRx = 0;
	PD_PHY.TxSop = 0x00u;  /* 以降の TX は SOP に戻す */
	if ( rxVDM->CommandType == 0x01 && rxVDM->Command == PD_VDM_DiscoverIdentity ) {
		VDM_Print_Cable_Identity_ACK("active probe");
	} else {
		printf("Cable Discover Identity: unexpected VDM cmd=0x%02X type=0x%02X\r\n",
		       (unsigned)rxVDM->Command, (unsigned)rxVDM->CommandType);
	}
	VDM_Cable_Probe_End();
}

static void VDM_Cable_DiscIdent_TxFailed(void)
{
	PD_PHY.WaitMsgRx = 0;
	PD_PHY.TxSop = 0x00u;
	printf("\r\nCable Discover Identity TX failed (no GoodCRC)\r\n");
	VDM_Cable_Probe_End();
}

static void VDM_Cable_DiscIdent_NoResponse(void)
{
	PD_PHY.WaitMsgRx = 0;
	PD_PHY.TxSop = 0x00u;
	printf("\r\nCable Discover Identity: no response (passive cable or no E-marker)\r\n");
	VDM_Cable_Probe_End();
}

static void VDM_Cable_Send_DiscIdent(void)
{
	printf("TX SOP' Discover Identity (cable / E-marker)\r\n");
	VDM_Cable_Enable_Vconn();
	Delay_Ms(5);  /* VCONN 安定待ち */
	txVDM->SVID           = 0xFF00;
	txVDM->Type           = 1;
	txVDM->VersionMajor   = 1;
	txVDM->VersionMinor   = 0;
	txVDM->ObjectPosition = 0;
	txVDM->CommandType    = 0;
	txVDM->Reserved       = 0;
	txVDM->Command        = (u16)PD_VDM_DiscoverIdentity;
	PD_PHY.TxSop = SOP_TX_SOPP;
	PD_PHY_Header_Init(5, 1, PD_Data_VendorDefined);
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 450;
	PD_Prot_pSet( NULL , VDM_Cable_DiscIdent_RX , VDM_Cable_DiscIdent_TxFailed , VDM_Cable_DiscIdent_NoResponse );
}

static void __attribute__((unused)) VDM_Cable_VconnSwap_RX(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_Accept ) {
		printf("VCONN_Swap: Accepted — querying cable\r\n");
		VDM_Cable_Send_DiscIdent();
	} else if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
	            ( rxHeader->MsgType == PD_Ctrl_Reject || rxHeader->MsgType == PD_Ctrl_NotSupported ) ) {
		printf("VCONN_Swap: Reject/NotSupported — source keeps VCONN; skip active SOP' (passive sniff only)\r\n");
		VDM_Cable_Probe_End();
	} else if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
	} else {
		printf("VCONN_Swap: unexpected 0x%02X — skip active cable query\r\n", rxHeader->MsgType);
		VDM_Cable_Probe_End();
	}
}

/*
 * SOP' Cable プローブ開始。VCONN_Swap → SOP' Discover Identity。
 * 1 = プローブ開始 (非同期), 0 = スキップ (既に試行済み or 契約なし)
 */
u8 PD_Cable_Probe_TryStart(void)
{
	if ( PD_Cable_Probe_Done ) return 0u;
	if ( VDM_State.Explicit_Contract_Established == 0 ) return 0u;

	PD_Cable_Probe_Done = 1;
	if ( s_cable_identity_logged ) {
		printf("Cable Identity already received (passive SOP'); skipping active probe\r\n");
		return 0u;
	}

	printf("Cable active probe disabled; waiting for passive SOP' Discover Identity ACK\r\n");
	return 0u;
}

void VDM_Reset_Disc_Probe_Only(void)
{
	VDM_Disc_SVID_Count = 0;
	VDM_Disc_Mode_Index = 0;
}

void VDM_Reset_Cable_State(void)
{
	s_cable_identity_logged = 0;
	PD_Cable_Probe_Done     = 0;
	s_vbus_seen_on          = 0;
	PD_Cable_Sniff_Stop();
}

void VDM_Sniff_SOPP_Cable(void)
{
	VDM_Try_Passive_SOPP();
}

/* ── Discover SVIDs タイムアウト ── */
static void VDM_Disc_SvidTimeout(void)
{
	PD_PHY.WaitMsgRx = 0;
	printf("\r\nDiscover SVIDs: no response (source ignored request — normal for power-only DFPs)\r\n");
	VDM_Source_Probe_Finish("Discover SVIDs timeout");
}

/* ── Discover Modes タイムアウト ── */
static void VDM_Disc_ModeTimeout(void)
{
	printf("\r\n");  /* PD_PHY_Header_Init の "tx" を行として完結させる */
	PD_PHY.WaitMsgRx = 0;
	if ( VDM_Disc_Mode_Index < VDM_Disc_SVID_Count ) {
		printf("Discover Modes timeout for SVID:0x%04X; skip\r\n",
			VDM_Disc_SVIDs[VDM_Disc_Mode_Index]);
	}
	VDM_Disc_Mode_Index++;
	pProt_TX_DISC_MODES_Next();
}

/* ── Discover SVIDs 送信 ── */
/*
 * WaitMsgRx=1 + pProt_RX_VDM で ACK を受け取る。
 * pRxTimeout=VDM_Disc_SvidTimeout で無応答を検出して IDLE へ戻る（約 30ms）。
 * 多くの充電器は UFP→DFP の SVIDs を無視するため、タイムアウト経路が主となる。
 */
void pProt_TX_DISC_SVIDS(void)
{
	VDM_Disc_SVID_Count = 0;
	VDM_Disc_Mode_Index = 0;
	printf("TX Discover SVIDs\r\n");
	txVDM->SVID          = 0xFF00;
	txVDM->Type          = 1;
	txVDM->VersionMajor  = 1;
	txVDM->VersionMinor  = 0;
	txVDM->ObjectPosition= 0;
	txVDM->CommandType   = 0;
	txVDM->Reserved      = 0;
	txVDM->Command       = (u16)PD_VDM_DiscoverSVIDs;
	PD_PHY_Header_Init(5, 1, PD_Data_VendorDefined);
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 450;
	PD_Prot_pSet( NULL , pProt_RX_VDM , VDM_Disc_SvidTimeout , VDM_Disc_SvidTimeout );
}

/* ── Discover Modes 送信 (VDM_Disc_Mode_Index が指す SVID 宛) ── */
/* WaitMsgRx=1 + pProt_RX_VDM で ACK/タイムアウトを処理する */
void pProt_TX_DISC_MODES_Next(void)
{
	u16 svid;
	if ( VDM_Disc_Mode_Index >= VDM_Disc_SVID_Count ) {
		printf("Discover Modes complete (all %d SVIDs)\r\n", VDM_Disc_SVID_Count);
		VDM_Source_Probe_Finish("all SVID modes visited");
		return;
	}
	svid = VDM_Disc_SVIDs[VDM_Disc_Mode_Index];
	printf("TX Discover Modes SVID:0x%04X [%d/%d]\r\n",
		svid, VDM_Disc_Mode_Index + 1, VDM_Disc_SVID_Count);
	txVDM->SVID          = svid;
	txVDM->Type          = 1;
	txVDM->VersionMajor  = 1;
	txVDM->VersionMinor  = 0;
	txVDM->ObjectPosition= 0;
	txVDM->CommandType   = 0;
	txVDM->Reserved      = 0;
	txVDM->Command       = (u16)PD_VDM_DiscoverModes;
	PD_PHY_Header_Init(5, 1, PD_Data_VendorDefined);
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 450;
	PD_Prot_pSet( NULL , pProt_RX_VDM , VDM_Disc_ModeTimeout , VDM_Disc_ModeTimeout );
}

/* ── SVID/Modes 状態リセット (デバイス切断時に PD_Prot.c から呼ぶ) ── */
void VDM_Reset_Disc_State(void)
{
	VDM_Reset_Disc_Probe_Only();
	VDM_Reset_Cable_State();
}

static void VDM_Source_Probe_Finish(const char *reason)
{
	PD_PHY.WaitMsgTx = 0;
	PD_PHY.WaitMsgRx = 0;
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	printf("SOP source discovery complete: %s\r\n", reason);
}

/* ── 固定の VDM 応答データ ── */
u8 VDM_Ident_PD2[] = {
	0x86, 0x1A, 0x00, 0x6C,
	0xE6, 0x36, 0x00, 0x00,
	0x00, 0x00, 0x35, 0xF0,
	0x08, 0x00, 0x00, 0x11
};
u8 VDM_Ident_PD3[] = {
	0x86, 0x1A, 0x40, 0x54,
	0xE6, 0x36, 0x00, 0x00,
	0x00, 0x00, 0x35, 0xF0,
	0x48, 0x00, 0x00, 0x01
};
u8 VDM_SVID[] = { 0x00, 0x00, 0x01, 0xFF };
u8 VDM_Mode[] = { 0x05, 0x0C, 0x00, 0x00 };

/* ── ディスパッチテーブル ── */
void (*VDM_REQ_Msg_Handle[][2])() = {
								/* DFP */				/* UFP */
	/* DISC_IDENT  */	pProt_RX_REQ_IDENT_DFP,		pProt_RX_REQ_IDENT_UFP,
	/* DISC_SVID   */	pProt_RX_REQ_SVID_DFP,		pProt_RX_REQ_SVID_UFP,
	/* DISC_MODE   */	pProt_RX_REQ_MODE_DFP,		pProt_RX_REQ_MODE_UFP,
	/* ENTER_MODE  */	pProt_TX_NAK,				pProt_RX_REQ_ENTER_UFP,
	/* EXIT_MODE   */	pProt_TX_NAK,				pProt_RX_REQ_EXIT_UFP,
	/* ATTENTION   */	pProt_RX_ATTENTION_DFP,		pProt_RX_ATTENTION_UFP,
};

void (*VDM_ACK_Msg_Handle[][2])() = {
								/* DFP */				/* UFP */
	/* DISC_IDENT  */	pProt_RX_ACK_IDENT_DFP,		pProt_RX_ACK_IDENT_UFP,
	/* DISC_SVID   */	pProt_RX_ACK_SVID_DFP,		pProt_RX_ACK_SVID_UFP,
	/* DISC_MODE   */	pProt_RX_ACK_MODE_DFP,		pProt_RX_ACK_MODE_UFP,
	/* ENTER_MODE  */	pProt_RX_ACK_ENTER_DFP,		NULL,
	/* EXIT_MODE   */	pProt_RX_ACK_EXIT_DFP,		NULL,
	/* ATTENTION   */	pProt_RX_ATTENTION_DFP,		pProt_RX_ATTENTION_UFP,
};

/*********************************************************************
 * @fn      pProt_RX_VDM
 * @brief   Receive VDM.
 *********************************************************************/
static void VDM_Try_Passive_SOPP(void)
{
	if ( PD_PHY.LastRxSop != SOP_RX_SOPP ) return;
	if ( rxHeader->MsgType != PD_Data_VendorDefined ) return;
	if ( s_cable_identity_logged ) return;
	if ( rxHeader->NDO < 4 ) return;
	if ( rxVDM->CommandType != 0x01 ||
	     rxVDM->Command != PD_VDM_DiscoverIdentity ) return;
	VDM_Print_Cable_Identity_ACK("passive sniff");
}

void pProt_RX_VDM(void)
{
	/* SOP' (cable): 契約前 (接続直後) や EPR 中のソース発クエリも傍受 */
	VDM_Try_Passive_SOPP();
	if ( PD_PHY.LastRxSop != SOP_RX_SOP ) return;

	if ( VDM_State.Explicit_Contract_Established == 0 ) return;
	if ( rxHeader->Extended || rxHeader->MsgType != PD_Data_VendorDefined || rxHeader->NDO == 0 ) {
		if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
		     rxHeader->MsgType == PD_Ctrl_SoftReset ) {
			printf("SOP source discovery interrupted by Soft_Reset\r\n");
			pProt_RX_SoftRst();
		} else if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
		            ( rxHeader->MsgType == PD_Ctrl_Reject ||
		              rxHeader->MsgType == PD_Ctrl_NotSupported ) ) {
			VDM_Source_Probe_Finish("source rejected discovery");
		} else {
			VDM_Source_Probe_Finish("unexpected SOP response");
		}
		return;
	}
	if ( rxVDM->Command < PD_VDM_DiscoverIdentity ||
	     rxVDM->Command > PD_VDM_Attention ) {
		printf("RX malformed VDM command:0x%02X\r\n", (unsigned)rxVDM->Command);
		VDM_Source_Probe_Finish("invalid VDM command");
		return;
	}

	if ( rxVDM->CommandType == 0x00 ) {
		if ( rxVDM->Command > PD_VDM_Attention ) PD_TX_NotSupported();
		else {
			if ( rxVDM->Type ) {
				(VDM_REQ_Msg_Handle[(rxVDM->Command)-1][(PD_PHY.Header.PortDataRole)?(0):(1)])();
			}
			else {
				if ( (!PD_PHY.Header.PortDataRole) && (rxVDM->SVID == 886) ) {}
				else {
					if ( (rxHeader->SpecRevision) == 1 ) {}
					else { PD_TX_NotSupported(); }
				}
			}
		}
	}
	else if ( rxVDM->CommandType == 0x01 ) {
		void (*handler)(void) = VDM_ACK_Msg_Handle[(rxVDM->Command)-1]
		                                          [(PD_PHY.Header.PortDataRole)?(0):(1)];
		if ( handler ) handler();
		else VDM_Source_Probe_Finish("unexpected VDM ACK");
	}
	else if ( rxVDM->CommandType == 0x02 ) {
		VDM_Source_Probe_Finish("source returned VDM NAK");
	}
	else if ( rxVDM->CommandType == 0x03 ) {
		VDM_Source_Probe_Finish("source returned VDM BUSY");
	}
	else {
		VDM_Source_Probe_Finish("invalid VDM command type");
	}
}

/*--- UFP 側レスポンダ (我々が応答を返す) ---*/

void pProt_RX_REQ_IDENT_UFP(void)
{
	memcpy( &PD_TX_BUF[1], &rxVDM->Data, 4 );
	txVDM->CommandType = 1;
	if ( rxVDM->VersionMajor > 1 ) {
		txVDM->VersionMajor = (rxHeader->SpecRevision == PD_Rev3) ? 1 : 0;
	}
	if ( rxVDM->VersionMajor == 1 ) {
		memcpy( &PD_TX_BUF[3], VDM_Ident_PD3, sizeof(VDM_Ident_PD3) );
		PD_PHY_Header_Init(5, (sizeof(VDM_Ident_PD3)/4)+1, PD_Data_VendorDefined);
	}
	else {
		memcpy( &PD_TX_BUF[3], VDM_Ident_PD2, sizeof(VDM_Ident_PD2) );
		PD_PHY_Header_Init(5, (sizeof(VDM_Ident_PD2)/4)+1, PD_Data_VendorDefined);
	}
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

void pProt_RX_REQ_SVID_UFP(void)
{
	memcpy( &PD_TX_BUF[1], &rxVDM->Data, 4 );
	txVDM->CommandType = 1;
	if ( rxVDM->VersionMajor > 1 ) {
		txVDM->VersionMajor = (rxHeader->SpecRevision == PD_Rev3) ? 1 : 0;
	}
	memcpy( &PD_TX_BUF[3], VDM_SVID, sizeof(VDM_SVID) );
	PD_PHY_Header_Init(5, (sizeof(VDM_SVID)/4)+1, PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

void pProt_RX_REQ_MODE_UFP(void)
{
	memcpy( &PD_TX_BUF[1], &rxVDM->Data, 4 );
	txVDM->CommandType = 1;
	if ( rxVDM->VersionMajor > 1 ) {
		txVDM->VersionMajor = (rxHeader->SpecRevision == PD_Rev3) ? 1 : 0;
	}
	memcpy( &PD_TX_BUF[3], VDM_Mode, sizeof(VDM_Mode) );
	PD_PHY_Header_Init(5, (sizeof(VDM_Mode)/4)+1, PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

void pProt_RX_REQ_ENTER_UFP(void)
{
	VDM_State.Enter_Mode_already = 1;
	pProt_TX_ACK();
}

void pProt_RX_REQ_EXIT_UFP(void)
{
	if ( VDM_State.Enter_Mode_already == 1 ) {
		pProt_TX_ACK();
		VDM_State.Enter_Mode_already = 0;
	}
	else {
		pProt_TX_NAK();
	}
}

void pProt_RX_ATTENTION_UFP(void) {}

/*--- DFP 側レスポンダ (通常は NAK) ---*/

void pProt_RX_REQ_IDENT_DFP(void)  { pProt_TX_NAK(); }
void pProt_RX_REQ_SVID_DFP(void)   { pProt_TX_NAK(); }
void pProt_RX_REQ_MODE_DFP(void)   { pProt_TX_NAK(); }
void pProt_RX_ATTENTION_DFP(void)  {}

/*--- NAK / ACK ヘルパー ---*/

void pProt_TX_NAK(void)
{
	memcpy( &PD_TX_BUF[1], &rxVDM->Data, 4 );
	txVDM->CommandType = 2;
	if ( rxVDM->VersionMajor > 1 ) {
		txVDM->VersionMajor = (rxHeader->SpecRevision == PD_Rev3) ? 1 : 0;
	}
	PD_PHY_Header_Init(5, 1, PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

void pProt_TX_ACK(void)
{
	memcpy( &PD_TX_BUF[1], &rxVDM->Data, 4 );
	txVDM->CommandType = 1;
	if ( rxVDM->VersionMajor > 1 ) {
		txVDM->VersionMajor = (rxHeader->SpecRevision == PD_Rev3) ? 1 : 0;
	}
	PD_PHY_Header_Init(5, 1, PD_Data_VendorDefined);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*--- Discover Identity 送信 ---*/

static void pDiscIdent_TxFailed(void)
{
	/*
	 * Discover Identity TX 失敗 (GoodCRC なし): VDM SOP をサポートしない可能性あり。
	 * EPR 対応ソースの場合はここから EPR プローブを試みる。
	 */
	PD_PHY.WaitMsgRx = 0;
	printf("\r\nDiscover Identity TX failed (no GoodCRC): VDM/SOP may not be supported by this source.\r\n");
	VDM_Source_Probe_Finish("Discover Identity TX failed");
}

static void pDiscIdent_NoResponse(void)
{
	/* GoodCRC は受け取ったが ACK が返ってこなかった（~30ms タイムアウト） */
	PD_PHY.WaitMsgRx = 0;
	printf("\r\nDiscover Identity: no response (source ignored request)\r\n");
	VDM_Source_Probe_Finish("Discover Identity timeout");
}

void pProt_TX_DISC_IDENT(void)
{
	txVDM->SVID          = 0xFF00;
	txVDM->Type          = 1;
	txVDM->VersionMajor  = 1;
	txVDM->VersionMinor  = 0;
	txVDM->ObjectPosition= 0;
	txVDM->CommandType   = 0;
	txVDM->Reserved      = 0;
	txVDM->Command       = (u16)PD_VDM_DiscoverIdentity;
	PD_PHY_Header_Init(5, 1, PD_Data_VendorDefined);
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 450;
	PD_Prot_pSet( NULL , pProt_RX_VDM , pDiscIdent_TxFailed , pDiscIdent_NoResponse );
}

/*--- DFP 側イニシエータ ACK ハンドラ ---*/

/*
 * Discover Identity ACK を受け取ったら内容を表示し、
 * 続いて Discover SVIDs を送信してソース情報を掘り下げる。
 */
void pProt_RX_ACK_IDENT_DFP(void)
{
	VDM_Print_Discover_Identity_ACK("DFP");
	/* Identity 完了 → Discover SVIDs へチェーン */
	pProt_TX_DISC_SVIDS();
}

/*
 * Discover SVIDs ACK を受け取ったら SVID リストを解析・表示し、
 * 各 SVID に対して Discover Modes を順番に送信する。
 */
void pProt_RX_ACK_SVID_DFP(void)
{
	u8 i;
	u8 svid_vdo_count;

	PD_PHY.WaitMsgRx = 0;

	/* NAK/BUSY の場合はここには来ない(pProt_RX_VDM で無視) が念のため */
	if ( rxVDM->CommandType != 0x01 ) {
		printf("Discover SVIDs: unexpected CommandType=0x%02X; stop\r\n", rxVDM->CommandType);
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		return;
	}

	svid_vdo_count = (rxHeader->NDO > 0) ? (rxHeader->NDO - 1) : 0;
	printf("\r\nRX Discover SVIDs ACK NDO:%d VDOs:%d\r\n", rxHeader->NDO, svid_vdo_count);

	VDM_Disc_SVID_Count = 0;
	for ( i = 0; i < svid_vdo_count && VDM_Disc_SVID_Count < MAX_DISC_SVIDS; i++ ) {
		/*
		 * SVID VDO フォーマット (PD spec):
		 *   bits 31:16 = SVID(n)   … PD_RX_BUF[4 + 2*i]  (high 16)
		 *   bits 15:0  = SVID(n+1) … PD_RX_BUF[3 + 2*i]  (low 16), 0x0000 if last
		 */
		u16 svid_hi = PD_RX_BUF[4 + 2*i];
		u16 svid_lo = PD_RX_BUF[3 + 2*i];

		if ( svid_hi ) {
			printf("  SVID[%d]: 0x%04X", VDM_Disc_SVID_Count, svid_hi);
			VDM_Print_SVID_Name(svid_hi);
			printf("\r\n");
			VDM_Disc_SVIDs[VDM_Disc_SVID_Count++] = svid_hi;
		}
		if ( svid_lo && VDM_Disc_SVID_Count < MAX_DISC_SVIDS ) {
			printf("  SVID[%d]: 0x%04X", VDM_Disc_SVID_Count, svid_lo);
			VDM_Print_SVID_Name(svid_lo);
			printf("\r\n");
			VDM_Disc_SVIDs[VDM_Disc_SVID_Count++] = svid_lo;
		}
	}
	printf("Total SVIDs: %d\r\n", VDM_Disc_SVID_Count);

	if ( VDM_Disc_SVID_Count == 0 ) {
		printf("No SVIDs returned; Discover Modes skipped\r\n");
		VDM_Source_Probe_Finish("source returned no SVIDs");
		return;
	}

	/* Discover Modes を最初の SVID から開始 */
	VDM_Disc_Mode_Index = 0;
	pProt_TX_DISC_MODES_Next();
}

/*
 * Discover Modes ACK を受け取ったらモード VDO を表示し、
 * 次の SVID へ進む（Enter Mode は送信しない）。
 */
void pProt_RX_ACK_MODE_DFP(void)
{
	u8 i;
	u8 mode_count;
	u16 svid;

	PD_PHY.WaitMsgRx = 0;

	if ( rxVDM->CommandType != 0x01 ) {
		printf("Discover Modes SVID:0x%04X NAK/BUSY; skip\r\n", rxVDM->SVID);
		VDM_Disc_Mode_Index++;
		pProt_TX_DISC_MODES_Next();
		return;
	}

	svid       = rxVDM->SVID;
	mode_count = (rxHeader->NDO > 0) ? (rxHeader->NDO - 1) : 0;

	printf("\r\nRX Discover Modes ACK SVID:0x%04X modes:%d\r\n", svid, mode_count);

	for ( i = 0; i < mode_count; i++ ) {
		u32 vdo = ((u32)PD_RX_BUF[4 + 2*i] << 16) | PD_RX_BUF[3 + 2*i];
		printf("  Mode[%d] VDO:0x%08lX", i + 1, (unsigned long)vdo);

		if ( svid == 0xFF01 ) {
			/* DisplayPort Mode VDO (PD spec Table 6-38) */
			u8 receptacle   = (u8)((vdo >> 6) & 0x01);
			u8 usb3_1_gndr  = (u8)((vdo >> 4) & 0x01);
			u8 usb2_0       = (u8)((vdo >> 2) & 0x01);
			u8 dfp_pinassign= (u8)((vdo >> 8) & 0xFF);
			u8 ufp_pinassign= (u8)((vdo >> 16) & 0xFF);
			printf(" [DP receptacle=%d usb3.1=%d usb2.0=%d dfpPin=0x%02X ufpPin=0x%02X]\r\n",
				receptacle, usb3_1_gndr, usb2_0, dfp_pinassign, ufp_pinassign);
		}
		else if ( svid == 0x8087 ) {
			/* Intel Thunderbolt */
			printf(" [Intel TBT VDO]\r\n");
		}
		else {
			printf("\r\n");
		}
	}

	VDM_Disc_Mode_Index++;
	pProt_TX_DISC_MODES_Next();
}

/*--- ACK ハンドラ (UFP / 未使用 DFP) ---*/

/*
 * シンク（UFP）として動作中に Discover Identity ACK を受け取るケース。
 * PortDataRole = UFP のため DFP ハンドラではなくこちらが呼ばれる。
 * スニファーは常に自分から Discover Identity を発行するため、
 * DFP ハンドラと同様に Discover SVIDs へチェーンする。
 */
void pProt_RX_ACK_IDENT_UFP(void)
{
	VDM_Print_Discover_Identity_ACK("UFP");
	pProt_TX_DISC_SVIDS();
}

/*
 * SVIDs/Modes の ACK も Identity 同様、スニファーが自分から発行した場合は
 * PortDataRole によらず DFP ハンドラ（パース処理あり）へ委譲する。
 */
void pProt_RX_ACK_SVID_UFP(void)
{
	pProt_RX_ACK_SVID_DFP();
}

void pProt_RX_ACK_MODE_UFP(void)
{
	pProt_RX_ACK_MODE_DFP();
}

void pProt_RX_ACK_ENTER_DFP(void)
{
	/* スニファーは Enter Mode しない → ここには来ないが念のため */
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
}

void pProt_RX_ACK_EXIT_DFP(void)
{
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
}
