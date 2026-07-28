/********************************** (C) COPYRIGHT  *******************************
 * File Name          : PD_Prot.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "PD_Prot.h"
#include "PD_User.H"
#include "PD_VDM.h"

extern st_VDM VDM_State;
#define PD_ANALYZER_INITIAL_REQUEST_MA 0   /* 0 = request advertised current */
#define PD_ANALYZER_NEGOTIATE_CONTRACT 1
/*
 * ★ タイミング修正: Delay_Ms をゼロに設定。
 *   Source_Cap 受信から Request 送信まで printf も含めて tSenderResponse(30ms) 以内に
 *   収める必要があるため、ブロッキング遅延を完全に除去する。
 */
#define PD_ANALYZER_REQUEST_DELAY_MS 0
#define PD_ANALYZER_INFO_PROBES      1
#define PD_ANALYZER_PPS_PROBE        1
#define PD_ANALYZER_EPR_PRE_SINKCAP  1
#define PD_ANALYZER_PPS_TARGET_MV    9000u
#define PD_ANALYZER_EPR_OBJ5_PPS_PROBE 1
#define PD_ANALYZER_EPR_PPS_TARGET_MV  12000u
#define PD_ANALYZER_EPR_PPS_CURRENT_MA 500u
/*
 * ★ 最初の MsgID0 ゲートを廃止。
 *   このゲートが SoftReset ループを引き起こし、タイミング問題を複雑化していた。
 *   最初の Source_Cap で即座に Request する方が信頼性が高い。
 */
#define PD_ANALYZER_SKIP_FIRST_MSGID0_SRC_CAP 0

static u8  Discover_Identity_Sent = 0;
static u8  PD_InfoProbe_Step   = 0;
static u8  PD_InfoProbe_Active = 0;
static u8  PD_InfoProbe_Done   = 0;   /* GET_* probe completed or aborted by SoftReset */
static u8  PD_InfoProbe_TxRetry = 0;
static u8  PD_InfoProbe_NoResponseCnt = 0;
static u8  PD_InfoProbe_SoftRst_Recovery_5V_Cnt = 0;
/*
 * プローブ中に Source が SoftReset を返したステップをスキップするためのマスク。
 * ビット N がセットされている場合、ステップ N はスキップする。
 * 物理切断 (pDevice_Attached) でリセット。
 */
static u16 PD_InfoProbe_Skip_Mask   = 0;
static u8  PD_InfoProbe_SoftRst_Cnt = 0;   /* プローブ起因 SoftReset の累計回数 */
/*
 * SoftReset が 1 回でも起きたら全ステップ (1-8) をスキップ。
 * ソースが任意の 1 つの PD3.0 メッセージを拒否した場合、
 * 他のメッセージも拒否することがほとんどのため、1 回で全スキップが最適。
 */
#define    PD_PROBE_SOFTRST_MAX      1      /* この回数に達したらステップ 1-9 を全スキップ */
#define    PD_PROBE_SKIP_ALL_MASK    0x03FEu /* ビット 1-9 = ステップ 1-9 */

static u8 PD_Request_Fail_Count = 0;
static u8 PD_Request_Stop_After_Fail = 0;
static u8 PD_Request_Async_Rx_Count = 0;
static u16 PD_PostExit_Request_DelayMs = 0;
static u16 PD_Suppressed_SrcCap_Count = 0;
static u32 PD_Current_SrcCap_Fingerprint = 0;
static u32 PD_Failed_SrcCap_Fingerprint = 0;
/*
 * 最後にプローブ（Discover Identityを含む）を試みたソースの PDO フィンガープリント。
 * 同一ソースが再接続しても再プローブしないようにするため、pDevice_Attached ではリセットしない。
 * 別のソースが接続された場合（指紋変化）は自動的に再プローブされる。
 */
static u32 PD_Probed_SrcCap_Fingerprint = 0;
static u8  PD_EPR_Probe_Done = 0;    /* 同一接続中に EPR プローブを 1 回のみ実行するフラグ */
static u32 PD_EPR_Attempted_FP = 0;  /* EPR を試みた PDO セットのフィンガープリント
                                       * Soft/Hard Reset 中は保持して EPR 無限ループを防ぐ。
                                       * 物理 CC 切断時にクリアして同じ電源の再測定を許可する。 */
static u8 PD_Failed_SrcCap_NDO = 0;
static u32 PD_Request_SrcCap_Fingerprint = 0;
static u8 PD_Request_SrcCap_NDO = 0;

/*
 * Source_Cap の情報を契約確立まで遅延表示するための一時保存変数。
 * Request 送信前に printf するとタイミングが tSenderResponse を超えるため、
 * Accept/PS_RDY 受信後または Request 失敗時に表示する。
 */
static u8  PD_Stored_SrcCap_NDO;
static u8  PD_Stored_SrcCap_SpecRev;
static u8  PD_Stored_SrcCap_MsgID;
static u32 PD_Stored_Request_RDO;
static u8  PD_Stored_Request_ObjectPos;
static u32 PD_Stored_Selected_PDO;
static u8  PD_Stored_SrcCap_EprCap;

/* PPS Contract Probe 状態 */
static u8  PD_PPS_Probe_Done = 0;   /* 試行済みフラグ (pDevice_Unattached でリセット) */
static u8  s_pps_pdo_idx     = 0;   /* 1-based PPS APDO インデックス                 */
static u8  s_pps_orig_obj    = 0;   /* 元の Fixed 契約の ObjectPos                   */
static u16 s_pps_req_mv      = 0;   /* リクエスト電圧 (mV)                           */

/* EPR Enter 前の Sink_Capabilities 送信フラグ */
static u8  s_epr_sinkcap_sent = 0;  /* 送信済み (pDevice_Unattached + InfoProbe_Start でリセット) */

enum {
	PD_RESULT_STATUS_UNKNOWN = 0,
	PD_RESULT_STATUS_PENDING,
	PD_RESULT_STATUS_PASS,
	PD_RESULT_STATUS_FAIL,
	PD_RESULT_STATUS_NA,
	PD_RESULT_STATUS_UNSUPPORTED,
	PD_RESULT_STATUS_NO_RESPONSE,
	PD_RESULT_STATUS_TX_FAILED,
	PD_RESULT_STATUS_NAK,
	PD_RESULT_STATUS_BUSY,
	PD_RESULT_STATUS_REJECTED,
	PD_RESULT_STATUS_INVALID
};

enum {
	PD_RESULT_EXIT_UNKNOWN = 0,
	PD_RESULT_EXIT_ACK,
	PD_RESULT_EXIT_SOFT_RESET,
	PD_RESULT_EXIT_SOURCE_CAP,
	PD_RESULT_EXIT_TIMEOUT,
	PD_RESULT_EXIT_UNEXPECTED
};

#define PD_RESULT_DIRTY_ATTACH    0x0001u
#define PD_RESULT_DIRTY_SPR       0x0002u
#define PD_RESULT_DIRTY_EPR       0x0004u
#define PD_RESULT_DIRTY_IDENTITY  0x0008u
#define PD_RESULT_DIRTY_CABLE     0x0010u
#define PD_RESULT_DIRTY_PROTOCOL  0x0020u

typedef struct {
	u16 session;
	u8  active;
	u8  cc;
	u8  spr_pdo_count;
	u8  pps_count;
	u8  epr_capable;
	u32 spr_max_mw;
	u32 spr_pdo[7];
	u8  epr_object_count;
	u8  epr_complete;
	u8  epr_fixed_count;
	u16 epr_fixed_mv[4];
	u16 epr_fixed_ma[4];
	u16 avs_min_mv;
	u16 avs_max_mv;
	u8  avs_pdp_w;
	u32 epr_pdo[11];
	u8  identity_status;
	u8  source_probe_status;
	u16 source_vid;
	u16 source_pid;
	u32 source_id_header;
	u32 source_product_vdo;
	u8  cable_status;
	u8  cable_type;
	u8  cable_current_code;
	u8  cable_max_voltage_code;
	u8  cable_usb_speed;
	u16 cable_vid;
	u16 cable_pid;
	u32 cable_id_header;
	u32 cable_product_vdo;
	u32 cable_vdo;
	u8  spr_contract_status;
	u8  pps_probe_status;
	u8  epr_enter_status;
	u8  chunk_status;
	u8  epr_exit_status;
	u8  info_probe_status[10];
	u8  complete;
} st_PD_Result;

static st_PD_Result s_pd_result;
static u16 s_pd_result_session_seq;
static volatile u16 s_pd_result_dirty;

static const char *PD_Result_Status_Name(u8 status)
{
	switch ( status ) {
	case PD_RESULT_STATUS_PENDING:      return "pending";
	case PD_RESULT_STATUS_PASS:         return "pass";
	case PD_RESULT_STATUS_FAIL:         return "fail";
	case PD_RESULT_STATUS_NA:           return "na";
	case PD_RESULT_STATUS_UNSUPPORTED:  return "unsupported";
	case PD_RESULT_STATUS_NO_RESPONSE:  return "no_response";
	case PD_RESULT_STATUS_TX_FAILED:    return "tx_failed";
	case PD_RESULT_STATUS_NAK:          return "nak";
	case PD_RESULT_STATUS_BUSY:         return "busy";
	case PD_RESULT_STATUS_REJECTED:     return "rejected";
	case PD_RESULT_STATUS_INVALID:      return "invalid";
	default:                            return "unknown";
	}
}

static const char *PD_Result_Probe_Key(u8 step)
{
	switch ( step ) {
	case 1: return "revision";
	case 2: return "source_cap_ext";
	case 3: return "status";
	case 4: return "source_info";
	case 5: return "pps_status";
	case 6: return "country_codes";
	case 7: return "manufacturer_info";
	case 8: return "battery_cap";
	case 9: return "battery_status";
	default: return "unknown";
	}
}

static void PD_Result_RecordProbeStatus(u8 step, u8 status)
{
	if ( step == 0u || step > 9u ) return;
	s_pd_result.info_probe_status[step] = status;
	if ( !s_pd_result.active ) return;
	printf("@PD1,type=probe,session=%u,name=%s,status=%s\r\n",
	       (unsigned)s_pd_result.session, PD_Result_Probe_Key(step),
	       PD_Result_Status_Name(status));
}

static const char *PD_Result_Exit_Name(u8 status)
{
	switch ( status ) {
	case PD_RESULT_EXIT_ACK:         return "ack";
	case PD_RESULT_EXIT_SOFT_RESET:  return "soft_reset";
	case PD_RESULT_EXIT_SOURCE_CAP:  return "source_cap";
	case PD_RESULT_EXIT_TIMEOUT:     return "timeout";
	case PD_RESULT_EXIT_UNEXPECTED:  return "unexpected";
	default:                         return "unknown";
	}
}

static u16 PD_Result_Cable_Current_mA(u8 code)
{
	if ( code == 1u ) return 3000u;
	if ( code == 2u ) return 5000u;
	return 0u;
}

static u16 PD_Result_Cable_Max_mV(u8 code)
{
	static const u16 voltage_mv[] = { 20000u, 30000u, 40000u, 48000u };
	return voltage_mv[code & 0x03u];
}

static const char *PD_Result_Cable_Type_Name(u8 code)
{
	if ( code == 3u ) return "passive";
	if ( code == 4u ) return "active";
	return "unknown";
}

static const char *PD_Result_Cable_Speed_Name(u8 code)
{
	switch ( code ) {
	case 0u: return "usb2";
	case 1u: return "usb3_gen1";
	case 2u: return "usb4_gen2";
	case 3u: return "usb4_gen3";
	case 4u: return "usb4_gen4";
	default: return "unknown";
	}
}

static void PD_Result_Emit(u16 dirty, u8 final)
{
	u8 i;
	if ( !s_pd_result.active && !final ) return;

	if ( dirty & PD_RESULT_DIRTY_ATTACH ) {
		printf("@PD1,type=attach,session=%u,cc=%u\r\n",
		       (unsigned)s_pd_result.session, (unsigned)s_pd_result.cc);
	}
	if ( dirty & PD_RESULT_DIRTY_SPR ) {
		printf("@PD1,type=spr,session=%u,pdo_count=%u,pps_count=%u,max_mw=%lu,epr_capable=%u\r\n",
		       (unsigned)s_pd_result.session, (unsigned)s_pd_result.spr_pdo_count,
		       (unsigned)s_pd_result.pps_count, (unsigned long)s_pd_result.spr_max_mw,
		       (unsigned)s_pd_result.epr_capable);
		for ( i = 0u; i < s_pd_result.spr_pdo_count && i < 7u; i++ ) {
			printf("@PD1,type=spr_pdo,session=%u,index=%u,raw=%08lX\r\n",
			       (unsigned)s_pd_result.session, (unsigned)(i + 1u),
			       (unsigned long)s_pd_result.spr_pdo[i]);
		}
	}
	if ( dirty & PD_RESULT_DIRTY_EPR ) {
		printf("@PD1,type=epr,session=%u,object_count=%u,complete=%u,fixed_count=%u,avs_min_mv=%u,avs_max_mv=%u,pdp_w=%u\r\n",
		       (unsigned)s_pd_result.session, (unsigned)s_pd_result.epr_object_count,
		       (unsigned)s_pd_result.epr_complete, (unsigned)s_pd_result.epr_fixed_count,
		       (unsigned)s_pd_result.avs_min_mv, (unsigned)s_pd_result.avs_max_mv,
		       (unsigned)s_pd_result.avs_pdp_w);
		for ( i = 0u; i < s_pd_result.epr_fixed_count && i < 4u; i++ ) {
			printf("@PD1,type=epr_fixed,session=%u,index=%u,mv=%u,ma=%u\r\n",
			       (unsigned)s_pd_result.session, (unsigned)i,
			       (unsigned)s_pd_result.epr_fixed_mv[i],
			       (unsigned)s_pd_result.epr_fixed_ma[i]);
		}
		for ( i = 0u; i < s_pd_result.epr_object_count && i < 11u; i++ ) {
			printf("@PD1,type=epr_pdo,session=%u,index=%u,raw=%08lX\r\n",
			       (unsigned)s_pd_result.session, (unsigned)(i + 1u),
			       (unsigned long)s_pd_result.epr_pdo[i]);
		}
	}
	if ( dirty & PD_RESULT_DIRTY_IDENTITY ) {
		printf("@PD1,type=identity,session=%u,status=%s,probe=%s,vid=%04X,pid=%04X,id_header=%08lX,product_vdo=%08lX\r\n",
		       (unsigned)s_pd_result.session,
		       PD_Result_Status_Name(s_pd_result.identity_status),
		       PD_Result_Status_Name(s_pd_result.source_probe_status),
		       s_pd_result.source_vid, s_pd_result.source_pid,
		       (unsigned long)s_pd_result.source_id_header,
		       (unsigned long)s_pd_result.source_product_vdo);
	}
	if ( dirty & PD_RESULT_DIRTY_CABLE ) {
		u8 current_code = s_pd_result.cable_current_code;
		u8 max_voltage_code = s_pd_result.cable_max_voltage_code;
		u32 cable_vdo = s_pd_result.cable_vdo;
		printf("@PD1,type=cable,session=%u,status=%s,cable_type=%u,cable_type_name=%s,current_ma=%u,cable_current_a=%u,max_mv=%u,cable_max_voltage_v=%u,usb_speed=%u,usb_speed_name=%s,cable_vdo_version=%u,cable_latency=%u,cable_termination=%u,vid=%04X,pid=%04X,id_header=%08lX,product_vdo=%08lX,cable_vdo=%08lX\r\n",
		       (unsigned)s_pd_result.session,
		       PD_Result_Status_Name(s_pd_result.cable_status),
		       (unsigned)s_pd_result.cable_type,
		       PD_Result_Cable_Type_Name(s_pd_result.cable_type),
		       (unsigned)PD_Result_Cable_Current_mA(current_code),
		       (unsigned)(PD_Result_Cable_Current_mA(current_code) / 1000u),
		       (unsigned)PD_Result_Cable_Max_mV(max_voltage_code),
		       (unsigned)(PD_Result_Cable_Max_mV(max_voltage_code) / 1000u),
		       (unsigned)s_pd_result.cable_usb_speed,
		       PD_Result_Cable_Speed_Name(s_pd_result.cable_usb_speed),
		       (unsigned)((cable_vdo >> 21) & 0x07u),
		       (unsigned)((cable_vdo >> 13) & 0x0Fu),
		       (unsigned)((cable_vdo >> 11) & 0x03u),
		       s_pd_result.cable_vid, s_pd_result.cable_pid,
		       (unsigned long)s_pd_result.cable_id_header,
		       (unsigned long)s_pd_result.cable_product_vdo,
		       (unsigned long)s_pd_result.cable_vdo);
	}
	if ( dirty & PD_RESULT_DIRTY_PROTOCOL ) {
		printf("@PD1,type=result,session=%u,spr=%s,pps=%s,epr=%s,chunk=%s,exit=%s,source_probe=%s,complete=%u,final=%u\r\n",
		       (unsigned)s_pd_result.session,
		       PD_Result_Status_Name(s_pd_result.spr_contract_status),
		       PD_Result_Status_Name(s_pd_result.pps_probe_status),
		       PD_Result_Status_Name(s_pd_result.epr_enter_status),
		       PD_Result_Status_Name(s_pd_result.chunk_status),
		       PD_Result_Exit_Name(s_pd_result.epr_exit_status),
		       PD_Result_Status_Name(s_pd_result.source_probe_status),
		       (unsigned)s_pd_result.complete, (unsigned)final);
	}
	if ( final ) {
		for ( i = 1u; i <= 9u; i++ ) {
			if ( s_pd_result.info_probe_status[i] != PD_RESULT_STATUS_UNKNOWN ) {
				printf("@PD1,type=probe,session=%u,name=%s,status=%s\r\n",
				       (unsigned)s_pd_result.session, PD_Result_Probe_Key(i),
				       PD_Result_Status_Name(s_pd_result.info_probe_status[i]));
			}
		}
	}
}

void PD_Result_OnAttach(void)
{
	u8 cable_status = s_pd_result.cable_status;
	u8 cable_type = s_pd_result.cable_type;
	u8 cable_current_code = s_pd_result.cable_current_code;
	u8 cable_max_voltage_code = s_pd_result.cable_max_voltage_code;
	u8 cable_usb_speed = s_pd_result.cable_usb_speed;
	u16 cable_vid = s_pd_result.cable_vid;
	u16 cable_pid = s_pd_result.cable_pid;
	u32 cable_id_header = s_pd_result.cable_id_header;
	u32 cable_product_vdo = s_pd_result.cable_product_vdo;
	u32 cable_vdo = s_pd_result.cable_vdo;
	if ( s_pd_result.active ) return;
	memset(&s_pd_result, 0, sizeof(s_pd_result));
	s_pd_result_session_seq++;
	if ( s_pd_result_session_seq == 0u ) s_pd_result_session_seq = 1u;
	s_pd_result.session = s_pd_result_session_seq;
	s_pd_result.active = 1u;
	s_pd_result.pps_probe_status = PD_RESULT_STATUS_NA;
	s_pd_result.epr_enter_status = PD_RESULT_STATUS_NA;
	s_pd_result.chunk_status = PD_RESULT_STATUS_NA;
	s_pd_result_dirty = PD_RESULT_DIRTY_ATTACH | PD_RESULT_DIRTY_PROTOCOL;
	/* The source can query SOP' before the sink attach state is entered. */
	if ( cable_status == PD_RESULT_STATUS_PASS ) {
		s_pd_result.cable_status = cable_status;
		s_pd_result.cable_type = cable_type;
		s_pd_result.cable_current_code = cable_current_code;
		s_pd_result.cable_max_voltage_code = cable_max_voltage_code;
		s_pd_result.cable_usb_speed = cable_usb_speed;
		s_pd_result.cable_vid = cable_vid;
		s_pd_result.cable_pid = cable_pid;
		s_pd_result.cable_id_header = cable_id_header;
		s_pd_result.cable_product_vdo = cable_product_vdo;
		s_pd_result.cable_vdo = cable_vdo;
		s_pd_result_dirty |= PD_RESULT_DIRTY_CABLE;
	}
}

void PD_Result_SetCC(u8 cc)
{
	if ( !s_pd_result.active ) return;
	s_pd_result.cc = cc;
	s_pd_result_dirty |= PD_RESULT_DIRTY_ATTACH;
}

void PD_Result_OnDetach(void)
{
	if ( !s_pd_result.active ) return;
	PD_Result_Emit(PD_RESULT_DIRTY_ATTACH | PD_RESULT_DIRTY_SPR |
	               PD_RESULT_DIRTY_EPR | PD_RESULT_DIRTY_IDENTITY |
	               PD_RESULT_DIRTY_CABLE | PD_RESULT_DIRTY_PROTOCOL, 1u);
	printf("@PD1,type=detach,session=%u\r\n", (unsigned)s_pd_result.session);
	memset(&s_pd_result, 0, sizeof(s_pd_result));
	s_pd_result_dirty = 0u;
}

void PD_Result_Poll(void)
{
	u16 dirty;
	if ( !s_pd_result.active || s_pd_result_dirty == 0u ) return;
	if ( PD_PHY.WaitMsgTx || PD_PHY.WaitMsgRx || (USBPD->CONTROL & PD_TX_EN) ) return;
	dirty = s_pd_result_dirty;
	s_pd_result_dirty = 0u;
	PD_Result_Emit(dirty, 0u);
}

static void PD_Result_UpdateSPR(const u32 *pdo, u8 count)
{
	u8 i;
	u32 max_mw = 0u;
	if ( count > 7u ) count = 7u;
	s_pd_result.spr_pdo_count = count;
	s_pd_result.pps_count = 0u;
	s_pd_result.epr_capable = 0u;
	memset(s_pd_result.spr_pdo, 0, sizeof(s_pd_result.spr_pdo));
	for ( i = 0u; i < count; i++ ) {
		u32 raw = pdo[i];
		u8 type = (u8)(raw >> 30);
		u32 mw = 0u;
		s_pd_result.spr_pdo[i] = raw;
		if ( type == 0u ) {
			u32 mv = ((raw >> 10) & 0x3FFu) * 50u;
			u32 ma = (raw & 0x3FFu) * 10u;
			mw = mv * ma / 1000u;
			if ( (raw >> 23) & 1u ) s_pd_result.epr_capable = 1u;
		} else if ( type == 1u ) {
			mw = (raw & 0x3FFu) * 250u;
		} else if ( type == 2u ) {
			u32 mv = ((raw >> 20) & 0x3FFu) * 50u;
			u32 ma = (raw & 0x3FFu) * 10u;
			mw = mv * ma / 1000u;
		} else if ( ((raw >> 28) & 0x03u) == 0u ) {
			u32 mv = ((raw >> 17) & 0xFFu) * 100u;
			u32 ma = (raw & 0x7Fu) * 50u;
			s_pd_result.pps_count++;
			mw = mv * ma / 1000u;
		}
		if ( mw > max_mw ) max_mw = mw;
	}
	s_pd_result.spr_max_mw = max_mw;
	if ( s_pd_result.pps_count ) {
		if ( s_pd_result.pps_probe_status == PD_RESULT_STATUS_NA )
			s_pd_result.pps_probe_status = PD_RESULT_STATUS_PENDING;
	} else {
		s_pd_result.pps_probe_status = PD_RESULT_STATUS_NA;
	}
	if ( s_pd_result.epr_capable ) {
		if ( s_pd_result.epr_enter_status == PD_RESULT_STATUS_NA )
			s_pd_result.epr_enter_status = PD_RESULT_STATUS_PENDING;
	} else {
		s_pd_result.epr_enter_status = PD_RESULT_STATUS_NA;
		s_pd_result.chunk_status = PD_RESULT_STATUS_NA;
	}
	s_pd_result_dirty |= PD_RESULT_DIRTY_SPR | PD_RESULT_DIRTY_PROTOCOL;
}

static void PD_Result_UpdateEPR(const u32 *pdo, u8 count, u8 complete)
{
	u8 i;
	if ( count > 11u ) count = 11u;
	s_pd_result.epr_object_count = count;
	s_pd_result.epr_complete = complete;
	s_pd_result.epr_fixed_count = 0u;
	s_pd_result.avs_min_mv = 0u;
	s_pd_result.avs_max_mv = 0u;
	s_pd_result.avs_pdp_w = 0u;
	memset(s_pd_result.epr_pdo, 0, sizeof(s_pd_result.epr_pdo));
	for ( i = 0u; i < count; i++ ) {
		u32 raw = pdo[i];
		s_pd_result.epr_pdo[i] = raw;
		if ( i >= 7u && raw != 0u ) {
			u8 type = (u8)(raw >> 30);
			if ( type == 0u && s_pd_result.epr_fixed_count < 4u ) {
				u8 n = s_pd_result.epr_fixed_count++;
				s_pd_result.epr_fixed_mv[n] = (u16)(((raw >> 10) & 0x3FFu) * 50u);
				s_pd_result.epr_fixed_ma[n] = (u16)((raw & 0x3FFu) * 10u);
			} else if ( type == 3u && (((raw >> 28) & 0x03u) == 1u ||
			                              ((raw >> 28) & 0x03u) == 2u) ) {
				s_pd_result.avs_min_mv = (u16)(((raw >> 8) & 0x1FFu) * 100u);
				s_pd_result.avs_max_mv = (u16)(((raw >> 17) & 0x1FFu) * 100u);
				s_pd_result.avs_pdp_w = (u8)(raw & 0xFFu);
			}
		}
	}
	s_pd_result.epr_enter_status = complete ? PD_RESULT_STATUS_PASS : s_pd_result.epr_enter_status;
	s_pd_result.chunk_status = complete ? PD_RESULT_STATUS_PASS : PD_RESULT_STATUS_FAIL;
	s_pd_result_dirty |= PD_RESULT_DIRTY_EPR | PD_RESULT_DIRTY_PROTOCOL;
}

void PD_Result_SetSourceIdentity(u16 vid, u16 pid, u32 id_header, u32 product_vdo)
{
	s_pd_result.identity_status = PD_RESULT_STATUS_PASS;
	s_pd_result.source_vid = vid;
	s_pd_result.source_pid = pid;
	s_pd_result.source_id_header = id_header;
	s_pd_result.source_product_vdo = product_vdo;
	s_pd_result_dirty |= PD_RESULT_DIRTY_IDENTITY;
}

void PD_Result_SetCableIdentity(u16 vid, u16 pid, u8 product_type,
		u8 current_code, u8 max_voltage_code, u8 usb_speed,
		u32 id_header, u32 product_vdo, u32 cable_vdo)
{
	s_pd_result.cable_status = PD_RESULT_STATUS_PASS;
	s_pd_result.cable_vid = vid;
	s_pd_result.cable_pid = pid;
	s_pd_result.cable_type = product_type;
	s_pd_result.cable_current_code = current_code;
	s_pd_result.cable_max_voltage_code = max_voltage_code;
	s_pd_result.cable_usb_speed = usb_speed;
	s_pd_result.cable_id_header = id_header;
	s_pd_result.cable_product_vdo = product_vdo;
	s_pd_result.cable_vdo = cable_vdo;
	s_pd_result_dirty |= PD_RESULT_DIRTY_CABLE;
}

void PD_Result_SetSourceDiscovery(u8 result)
{
	switch ( result ) {
	case PD_RESULT_DISC_DONE:         s_pd_result.source_probe_status = PD_RESULT_STATUS_PASS; break;
	case PD_RESULT_DISC_NO_RESPONSE:  s_pd_result.source_probe_status = PD_RESULT_STATUS_NO_RESPONSE; break;
	case PD_RESULT_DISC_TX_FAILED:    s_pd_result.source_probe_status = PD_RESULT_STATUS_TX_FAILED; break;
	case PD_RESULT_DISC_NAK:          s_pd_result.source_probe_status = PD_RESULT_STATUS_NAK; break;
	case PD_RESULT_DISC_BUSY:         s_pd_result.source_probe_status = PD_RESULT_STATUS_BUSY; break;
	case PD_RESULT_DISC_REJECTED:     s_pd_result.source_probe_status = PD_RESULT_STATUS_REJECTED; break;
	default:                          s_pd_result.source_probe_status = PD_RESULT_STATUS_INVALID; break;
	}
	if ( s_pd_result.identity_status == PD_RESULT_STATUS_UNKNOWN )
		s_pd_result.identity_status = s_pd_result.source_probe_status;
	s_pd_result.complete = 1u;
	s_pd_result_dirty |= PD_RESULT_DIRTY_IDENTITY | PD_RESULT_DIRTY_PROTOCOL;
}

static void PD_Print_Source_Capabilities_Deferred(void);

#if !PD_ANALYZER_NEGOTIATE_CONTRACT
static u32 PD_Last_Printed_SrcCap_Fingerprint = 0;
static u8 PD_Last_Printed_SrcCap_NDO = 0;
static u8 PD_Last_Printed_SrcCap_Valid = 0;
static u16 PD_Suppressed_Same_SrcCap_Count = 0;
#endif

static void PD_InfoProbe_Start(void);
static void PD_InfoProbe_SendNext(void);
static void PD_InfoProbe_SendExt(u8 ext_type, u8 data_size, u8 d0, u8 d1);
static void PD_InfoProbe_RX(void);
static void PD_InfoProbe_Chunk_RX(void);
static void PD_InfoProbe_Chunk_Timeout(void);
static void PD_InfoProbe_TxFailed(void);
static void PD_InfoProbe_RxTimeout(void);
static void pProt_Request_TxFailed(void);
static void pProt_Request_RxTimeout(void);
static void pProt_PS_RDY_Failed(void);

void PD_Request_Arbiter_Tick(u8 delta_ms)
{
	if ( PD_PostExit_Request_DelayMs == 0u ) return;
	if ( !PD_DEVICE.ConnectStat ) {
		PD_PostExit_Request_DelayMs = 0u;
		return;
	}
	if ( PD_PostExit_Request_DelayMs > delta_ms ) {
		PD_PostExit_Request_DelayMs =
			(u16)(PD_PostExit_Request_DelayMs - delta_ms);
		return;
	}

	/*
	 * Poll drains deferred RX before this tick runs.  If a source-initiated
	 * VDM response is still queued or on the wire, wait another millisecond
	 * rather than overwrite its TX buffer with Request.
	 */
	if ( PD_PHY.WaitMsgTx || PD_PHY.WaitMsgRx ||
	     (USBPD->CONTROL & PD_TX_EN) != 0u ) {
		PD_PostExit_Request_DelayMs = 1u;
		return;
	}

	PD_PostExit_Request_DelayMs = 0u;
	printf("Source AMS guard complete; TX Request Fixed PDO1\r\n");
	pProt_TX_Request();
}

static u32 PD_SourceCap_Fingerprint(void)
{
	u8 i;
	u32 h = 0x811C9DC5;
	for ( i = 0; i < rxHeader->NDO; i++ ) {
		h ^= PD_PHY.rxSrcCap[i];
		h *= 16777619;
	}
	return h ^ rxHeader->NDO;
}
/* EPR_Source_Capabilities 再構成表示用: コンパクトな PDO 内容文字列を出力 (改行なし) */
static void PD_Print_PDO_Content_Compact(u32 pdo)
{
	u8 type = (u8)((pdo >> 30) & 0x03u);
	if ( type == 0u ) {
		u16 mv = (u16)(((pdo >> 10) & 0x3FFu) * 50u);
		u16 ma = (u16)((pdo & 0x3FFu) * 10u);
		u32 mw = (u32)mv * ma / 1000u;
		printf("Fixed %u.%uV/%u.%uA (%uW)",
		       (unsigned)(mv/1000u), (unsigned)((mv%1000u)/100u),
		       (unsigned)(ma/1000u), (unsigned)((ma%1000u)/100u),
		       (unsigned)(mw/1000u));
		if ( (pdo >> 23u) & 1u ) printf(" EPRCap");
	} else if ( type == 1u ) {
		u16 max_mv = (u16)(((pdo >> 20) & 0x3FFu) * 50u);
		u16 min_mv = (u16)(((pdo >> 10) & 0x3FFu) * 50u);
		u32 mw     = (u32)(pdo & 0x3FFu) * 250u;
		printf("Battery %u.%uV-%u.%uV (%uW)",
		       (unsigned)(min_mv/1000u), (unsigned)((min_mv%1000u)/100u),
		       (unsigned)(max_mv/1000u), (unsigned)((max_mv%1000u)/100u),
		       (unsigned)(mw/1000u));
	} else if ( type == 2u ) {
		u16 max_mv = (u16)(((pdo >> 20) & 0x3FFu) * 50u);
		u16 min_mv = (u16)(((pdo >> 10) & 0x3FFu) * 50u);
		u16 ma     = (u16)((pdo & 0x3FFu) * 10u);
		printf("Variable %u.%uV-%u.%uV/%u.%uA",
		       (unsigned)(min_mv/1000u), (unsigned)((min_mv%1000u)/100u),
		       (unsigned)(max_mv/1000u), (unsigned)((max_mv%1000u)/100u),
		       (unsigned)(ma/1000u),     (unsigned)((ma%1000u)/100u));
	} else {
		u8 apdo_type = (u8)((pdo >> 28) & 0x03u);
		if ( apdo_type == 0u ) {
			u16 max_mv = (u16)(((pdo >> 17) & 0xFFu)  * 100u);
			u16 min_mv = (u16)(((pdo >> 8)  & 0xFFu)  * 100u);
			u16 ma     = (u16)((pdo & 0x7Fu) * 50u);
			printf("PPS %u.%uV-%u.%uV/%u.%uA",
			       (unsigned)(min_mv/1000u), (unsigned)((min_mv%1000u)/100u),
			       (unsigned)(max_mv/1000u), (unsigned)((max_mv%1000u)/100u),
			       (unsigned)(ma/1000u),     (unsigned)((ma%1000u)/100u));
		} else if ( apdo_type == 1u || apdo_type == 2u ) {
			/* PD 3.2 Table 6-81: MaxVoltage = bits[25:17] (9 bits, 100mV)
			 * bit27 is Peak Overcurrent Support, bit26 is reserved/vendor flag.
			 * Mask must be 0x1FF, not 0x7FF, or AOHI-style PDOs with bit26=1 overflow u16. */
			u16 max_mv = (u16)(((pdo >> 17) & 0x1FFu) * 100u);
			u16 min_mv = (u16)(((pdo >> 8)  & 0x1FFu) * 100u);
			u8  pdp_w  = (u8)(pdo & 0xFFu);
			printf("EPR AVS %u.%uV-%u.%uV PDP=%uW",
			       (unsigned)(min_mv/1000u), (unsigned)((min_mv%1000u)/100u),
			       (unsigned)(max_mv/1000u), (unsigned)((max_mv%1000u)/100u),
			       (unsigned)pdp_w);
		} else {
			printf("APDO type:3 raw:0x%08lX", (unsigned long)pdo);
		}
	}
}

static void PD_Print_PDO(u8 index, u32 pdo)
{
	u8 type = (u8)((pdo >> 30) & 0x03);
	/* index 1-7 = SPR PDOs, index 8+ = EPR PDOs (EPR_Source_Capabilities objects) */
	printf("%s PDO[%d] raw:0x%08lX ", (index > 7) ? "EPR" : "SPR", index, (unsigned long)pdo);
	if ( type == 0 ) {
		u16 mv = (u16)(((pdo >> 10) & 0x3FF) * 50);
		u16 ma = (u16)((pdo & 0x3FF) * 10);
		u32 mw = ((u32)mv * ma) / 1000;
		printf("Fixed %umV %umA %lumW", mv, ma, (unsigned long)mw);
		printf(" flags: DRP=%d Suspend=%d Unconstr=%d USBComm=%d DRD=%d UnchunkExt=%d EPRCap=%d Peak=%lu",
			(int)((pdo >> 29) & 1), (int)((pdo >> 28) & 1), (int)((pdo >> 27) & 1),
			(int)((pdo >> 26) & 1), (int)((pdo >> 25) & 1), (int)((pdo >> 24) & 1),
			(int)((pdo >> 23) & 1), (unsigned long)((pdo >> 20) & 0x03));
	}
	else if ( type == 1 ) {
		u16 max_mv = (u16)(((pdo >> 20) & 0x3FF) * 50);
		u16 min_mv = (u16)(((pdo >> 10) & 0x3FF) * 50);
		u32 mw = (u32)(pdo & 0x3FF) * 250;
		printf("Battery %u-%umV %lumW", min_mv, max_mv, (unsigned long)mw);
	}
	else if ( type == 2 ) {
		u16 max_mv = (u16)(((pdo >> 20) & 0x3FF) * 50);
		u16 min_mv = (u16)(((pdo >> 10) & 0x3FF) * 50);
		u16 ma = (u16)((pdo & 0x3FF) * 10);
		printf("Variable %u-%umV %umA", min_mv, max_mv, ma);
	}
	else {
		u8 apdo_type = (u8)((pdo >> 28) & 0x03);
		if ( apdo_type == 0 ) {
			u16 max_mv = (u16)(((pdo >> 17) & 0xFF) * 100);
			u16 min_mv = (u16)(((pdo >> 8) & 0xFF) * 100);
			u16 ma = (u16)((pdo & 0x7F) * 50);
			printf("SPR PPS APDO %u-%umV %umA powerLimited=%d", min_mv, max_mv, ma, (int)((pdo >> 27) & 1));
		}
		else if ( apdo_type == 1 || apdo_type == 2 ) {
			/* bits[25:17]=MaxVoltage (9-bit, 100mV); bits[27:26]=flags (not voltage) */
			u16 max_mv = (u16)(((pdo >> 17) & 0x1FF) * 100);
			u16 min_mv = (u16)(((pdo >> 8)  & 0x1FF) * 100);
			u8  pdp_w  = (u8)(pdo & 0xFF);
			printf("EPR AVS APDO %u-%umV PDP=%uW", min_mv, max_mv, pdp_w);
			if ( min_mv == 0 || max_mv < 5000 || min_mv > max_mv ) {
				/* 非準拠値: EPR AVS として解釈できない。
				 * 上位 2 bit が誤って 11 に設定された Fixed PDO の可能性がある。
				 * bits[31:30] を 0 に強制して Fixed PDO として再解釈を試みる。 */
				u32  pdo_fixed = pdo & 0x3FFFFFFF;
				u16  f_mv  = (u16)(((pdo_fixed >> 10) & 0x3FF) * 50);
				u16  f_ma  = (u16)((pdo_fixed & 0x3FF) * 10);
				u32  f_mw  = (u32)f_mv * f_ma / 1000;
				if ( f_mv >= 4750 && f_mv <= 50000 && f_ma > 0 ) {
					printf(" (WARN: non-compliant as EPR AVS;"
					       " may be Fixed %umV %umA %lumW with wrong type-bits)",
					       f_mv, f_ma, f_mw);
				} else {
					printf(" (WARN: voltage fields appear non-compliant)");
				}
			}
		}
		else {
			printf("Augmented APDO type:%d (reserved/unknown)", apdo_type);
		}
	}
	printf("\r\n");
}

static void __attribute__((unused)) PD_Print_Source_Capabilities(void)
{
	u8 i;
	u32 max_mw = 0;
	u8 max_index = 0;
	u8 epr_capable = 0;
	u8 apdo_count = 0;
	printf("RX Source_Capabilities (SPR) NDO:%d SpecRev:%d MsgID:%d\r\n", rxHeader->NDO, rxHeader->SpecRevision, rxHeader->MsgID);
	for ( i = 0; i < rxHeader->NDO; i++ ) {
		u32 pdo = PD_PHY.rxSrcCap[i];
		u8 type = (u8)((pdo >> 30) & 0x03);
		u32 mw = 0;
		PD_Print_PDO((u8)(i + 1), pdo);
		if ( type == 0 ) {
			u32 mv = ((pdo >> 10) & 0x3FF) * 50;
			u32 ma = (pdo & 0x3FF) * 10;
			mw = (mv * ma) / 1000;
			if ( (pdo >> 23) & 1 ) {
				epr_capable = 1;
			}
		}
		else if ( type == 1 ) {
			mw = (pdo & 0x3FF) * 250;
		}
		else if ( type == 2 ) {
			u32 max_mv = ((pdo >> 20) & 0x3FF) * 50;
			u32 ma = (pdo & 0x3FF) * 10;
			mw = (max_mv * ma) / 1000;
		}
		else {
			apdo_count++;
			if ( ((pdo >> 28) & 0x03) == 0 ) {
				u32 max_mv = ((pdo >> 17) & 0xFF) * 100;
				u32 ma = (pdo & 0x7F) * 50;
				mw = (max_mv * ma) / 1000;
			}
		}
		if ( mw > max_mw ) {
			max_mw = mw;
			max_index = i + 1;
		}
	}
	printf("SPR summary: PDO count=%d APDO count=%d maxAdvertised=%lumW at SPR PDO[%d]\r\n",
		rxHeader->NDO, apdo_count, (unsigned long)max_mw, max_index);
	printf("EPR summary: EPRModeCapable=%d, Object 8+ EPR PDOs are not present in this SPR Source_Capabilities message\r\n", epr_capable);
	printf("EPR note: PD3.1 reserves Object 1-7 for SPR; EPR PDOs start at Object 8 after EPR Mode entry.\r\n");
}

/*
 * Source_Cap の内容を遅延表示する関数。
 * PD_PHY.rxSrcCap[] に保存済みのデータと PD_Stored_SrcCap_* 変数を使う。
 * 契約完了 (PS_RDY 受信) または Request 失敗時に呼ぶ。
 */
static void PD_Print_Source_Capabilities_Deferred(void)
{
	u8 i;
	u32 max_mw = 0;
	u8 max_index = 0;
	u8 epr_capable = PD_Stored_SrcCap_EprCap;
	u8 apdo_count = 0;

	PD_Result_UpdateSPR(PD_PHY.rxSrcCap, PD_Stored_SrcCap_NDO);

	printf("RX Source_Capabilities (SPR) NDO:%d SpecRev:%d MsgID:%d\r\n",
		PD_Stored_SrcCap_NDO, PD_Stored_SrcCap_SpecRev, PD_Stored_SrcCap_MsgID);
	for ( i = 0; i < PD_Stored_SrcCap_NDO; i++ ) {
		u32 pdo  = PD_PHY.rxSrcCap[i];
		u8  type = (u8)((pdo >> 30) & 0x03);
		u32 mw   = 0;
		PD_Print_PDO((u8)(i + 1), pdo);
		if ( type == 0 ) {
			u32 mv = ((pdo >> 10) & 0x3FF) * 50;
			u32 ma = (pdo & 0x3FF) * 10;
			mw = (mv * ma) / 1000;
		} else if ( type == 1 ) {
			mw = (pdo & 0x3FF) * 250;
		} else if ( type == 2 ) {
			u32 max_mv = ((pdo >> 20) & 0x3FF) * 50;
			u32 ma = (pdo & 0x3FF) * 10;
			mw = (max_mv * ma) / 1000;
		} else {
			apdo_count++;
			if ( ((pdo >> 28) & 0x03) == 0 ) {
				u32 max_mv = ((pdo >> 17) & 0xFF) * 100;
				u32 ma = (pdo & 0x7F) * 50;
				mw = (max_mv * ma) / 1000;
			}
		}
		if ( mw > max_mw ) { max_mw = mw; max_index = i + 1; }
	}
	printf("SPR summary: PDO count=%d APDO count=%d maxAdvertised=%lumW at SPR PDO[%d]\r\n",
		PD_Stored_SrcCap_NDO, apdo_count, (unsigned long)max_mw, max_index);
	printf("EPR summary: EPRModeCapable=%d, Object 8+ EPR PDOs are not present in this SPR Source_Capabilities message\r\n",
		epr_capable);
	printf("EPR note: PD3.1 reserves Object 1-7 for SPR; EPR PDOs start at Object 8 after EPR Mode entry.\r\n");
}

static void PD_Print_Request(u32 rdo, u32 selected_pdo)
{
	u8 obj = (u8)((rdo >> 28) & 0x07);
	u8 cap_mismatch = (u8)((rdo >> 26) & 0x01);
	u8 usb_comm = (u8)((rdo >> 25) & 0x01);
	u8 no_suspend = (u8)((rdo >> 24) & 0x01);
	u8 pdo_type = (u8)((selected_pdo >> 30) & 0x03);
	printf("TX Request RDO:0x%08lX ObjectPos(PDO Index):%d CapMismatch:%d USBComm:%d NoSuspend:%d\r\n",
		(unsigned long)rdo, obj, cap_mismatch, usb_comm, no_suspend);
	if ( pdo_type == 3 ) {
		u16 mv = (u16)(((rdo >> 9) & 0x7FF) * 20);
		u16 ma = (u16)((rdo & 0x7F) * 50);
		printf("Request PPS: %umV %umA\r\n", mv, ma);
	}
	else {
		u16 op_ma = (u16)((rdo & 0x3FF) * 10);
		u16 max_ma = (u16)(((rdo >> 10) & 0x3FF) * 10);
		printf("Request Fixed/Var: operating=%umA max=%umA\r\n", op_ma, max_ma);
	}
}


static const char *PD_Ctrl_Msg_Name(u8 msg)
{
	switch ( msg ) {
	case PD_Ctrl_GoodCRC:             return "GoodCRC";
	case PD_Ctrl_GotoMin:             return "GotoMin";
	case PD_Ctrl_Accept:              return "Accept";
	case PD_Ctrl_Reject:              return "Reject";
	case PD_Ctrl_Ping:                return "Ping";
	case PD_Ctrl_PS_Ready:            return "PS_RDY";
	case PD_Ctrl_GetSrcCap:           return "Get_Source_Cap";
	case PD_Ctrl_GetSinkCap:          return "Get_Sink_Cap";
	case PD_Ctrl_DRSwap:              return "DR_Swap";
	case PD_Ctrl_PRSwap:              return "PR_Swap";
	case PD_Ctrl_VconnSwap:           return "VCONN_Swap";
	case PD_Ctrl_Wait:                return "Wait";
	case PD_Ctrl_SoftReset:           return "SoftReset";
	case PD_Ctrl_DataReset:           return "DataReset";
	case PD_Ctrl_DataResetComplete:   return "DataReset_Complete";
	case PD_Ctrl_NotSupported:        return "NotSupported";
	case PD_Ctrl_GetSrcCapExtended:   return "Get_Source_Cap_Extended";
	case PD_Ctrl_GetStatus:           return "Get_Status";
	case PD_Ctrl_FRSwap:              return "FR_Swap";
	case PD_Ctrl_GetPPSStatus:        return "Get_PPS_Status";
	case PD_Ctrl_GetCountryCodes:     return "Get_Country_Codes";
	case PD_Ctrl_GetSinkCapExtended:  return "Get_Sink_Cap_Extended";
	case PD_Ctrl_GetSrcInfo:          return "Get_Source_Info";
	case PD_Ctrl_GetRevision:         return "Get_Revision";
	default: return "Control";
	}
}

static const char *PD_Data_Msg_Name(u8 msg)
{
	switch ( msg ) {
	case PD_Data_SrcCap: return "Source_Capabilities";
	case PD_Data_Request: return "Request";
	case PD_Data_BatteryStatus: return "Battery_Status";
	case PD_Data_Alert: return "Alert";
	case PD_Data_GetCountryInfo: return "Get_Country_Info";
	case PD_Data_EPRMode: return "EPR_Mode";
	case PD_Data_SrcInfo: return "Source_Info";
	case PD_Data_Revision: return "Revision";
	case PD_Data_VendorDefined: return "VDM";
	default: return "Data";
	}
}

static const char *PD_Ext_Msg_Name(u8 msg)
{
	switch ( msg ) {
	case PD_Ext_SrcCapExtended:     return "Source_Capabilities_Extended";
	case PD_Ext_Status:             return "Status";
	case PD_Ext_ManufacturerInfo:   return "Manufacturer_Info";
	case PD_Ext_PPSStatus:          return "PPS_Status";
	case PD_Ext_CountryInfo:        return "Country_Info";
	case PD_Ext_CountryCodes:       return "Country_Codes";
	case PD_Ext_SinkCapExtended:    return "Sink_Capabilities_Extended";
	case PD_Ext_BattertCap:         return "Battery_Capabilities";
	case PD_Ext_GetManufacturerInfo:return "Get_Manufacturer_Info";
	case PD_Ext_GetBatteryCap:      return "Get_Battery_Cap";
	case PD_Ext_GetBatteryStatus:   return "Get_Battery_Status";
	default: return "Extended";
	}
}

/*
 * プローブステップ番号とメッセージ名のマッピング。
 * 旧: step1=Get_Source_Cap_after_contract (削除) → Get_Revision が step2 だった
 * 新: step1=Get_Revision (最初に送る) → ソースのネゴシエーション状態を汚さない
 *
 * 【変更理由】
 * Get_Source_Cap (0x07) をコントラクト直後に送ると、安価なソースは
 * 「再ネゴシエーション開始」と解釈して Request を待つ状態に遷移する。
 * その状態で Get_Revision (0x18) を受け取ると「シーケンス違反」として
 * SoftReset を返す。Get_Source_Cap を省いて Get_Revision を最初に送ると
 * ソースの状態機が汚れていないため正しく応答できる可能性が高い。
 * (旧ファームの成功スクリーンショットも Get_Revision が最初だった)
 */
static const char *PD_InfoProbe_Name(u8 step)
{
	switch ( step ) {
	case 1: return "Get_Revision";
	case 2: return "Get_Source_Cap_Extended";
	case 3: return "Get_Status";
	case 4: return "Get_Source_Info";
	case 5: return "Get_PPS_Status";
	case 6: return "Get_Country_Codes";
	case 7: return "Get_Manufacturer_Info";
	case 8: return "Get_Battery_Cap";
	case 9: return "Get_Battery_Status";
	default: return "Done";
	}
}

static u8 PD_InfoProbe_CtrlMsg(u8 step)
{
	switch ( step ) {
	case 1: return PD_Ctrl_GetRevision;
	case 2: return PD_Ctrl_GetSrcCapExtended;
	case 3: return PD_Ctrl_GetStatus;
	case 4: return PD_Ctrl_GetSrcInfo;
	case 5: return PD_Ctrl_GetPPSStatus;
	case 6: return PD_Ctrl_GetCountryCodes;
	default: return 0;
	}
}

static u8 PD_Is_Get_Request_Control(u8 msg_type)
{
	switch ( msg_type ) {
	case PD_Ctrl_GetSrcCap:
	case PD_Ctrl_GetSinkCap:
	case PD_Ctrl_GetSrcCapExtended:
	case PD_Ctrl_GetStatus:
	case PD_Ctrl_GetPPSStatus:
	case PD_Ctrl_GetCountryCodes:
	case PD_Ctrl_GetSinkCapExtended:
	case PD_Ctrl_GetSrcInfo:
	case PD_Ctrl_GetRevision:
		return 1;
	default:
		return 0;
	}
}

static u8 PD_Is_Get_Request_Extended(u8 msg_type)
{
	switch ( msg_type ) {
	case PD_Ext_GetBatteryCap:
	case PD_Ext_GetBatteryStatus:
	case PD_Ext_GetManufacturerInfo:
		return 1;
	default:
		return 0;
	}
}

static u8 PD_Is_Extended_Chunk0_With_More_Data(void)
{
	return (u8)( rxHeader->Extended
	          && rxExtHeader->Chunked
	          && rxExtHeader->ChunkNumber == 0
	          && rxExtHeader->RequestChunk == 0
	          && rxExtHeader->DataSize > 26u );
}

static u8 PD_InfoProbe_Response_Matches(u8 step)
{
	if ( PD_PHY.LastRxSop != PD_PHY_RX_SOP ) return 0u;

	switch ( step ) {
	case 1:
		return (u8)( !rxHeader->Extended && rxHeader->NDO >= 1u
		          && rxHeader->MsgType == PD_Data_Revision );
	case 2:
		return (u8)( rxHeader->Extended
		          && rxHeader->MsgType == PD_Ext_SrcCapExtended );
	case 3:
		return (u8)( rxHeader->Extended
		          && rxHeader->MsgType == PD_Ext_Status );
	case 4:
		return (u8)( !rxHeader->Extended && rxHeader->NDO >= 1u
		          && rxHeader->MsgType == PD_Data_SrcInfo );
	case 5:
		return (u8)( rxHeader->Extended
		          && rxHeader->MsgType == PD_Ext_PPSStatus );
	case 6:
		return (u8)( rxHeader->Extended
		          && rxHeader->MsgType == PD_Ext_CountryCodes );
	case 7:
		return (u8)( rxHeader->Extended
		          && rxHeader->MsgType == PD_Ext_ManufacturerInfo );
	case 8:
		return (u8)( rxHeader->Extended
		          && rxHeader->MsgType == PD_Ext_BattertCap );
	case 9:
		return (u8)( !rxHeader->Extended && rxHeader->NDO >= 1u
		          && rxHeader->MsgType == PD_Data_BatteryStatus );
	default:
		return 0u;
	}
}

static void PD_Print_Raw_RX(const char *prefix)
{
	u8 i;
	u8 words = 1 + rxHeader->NDO * 2;
	u8 bytes = 2 + rxHeader->NDO * 4;
	u8 *raw = (u8 *)PD_RX_BUF;
	if ( rxHeader->Extended ) {
		bytes += 2;
	}
	if ( words > 16 ) {
		words = 16;
	}
	if ( bytes > 64 ) {
		bytes = 64;
	}
	printf("%s RAW16:", prefix);
	for ( i = 0; i < words; i++ ) {
		printf(" [%d]=%04X", i, PD_RX_BUF[i]);
	}
	printf("\r\n");
	printf("%s RAW8:", prefix);
	for ( i = 0; i < bytes; i++ ) {
		printf(" %02X", raw[i]);
	}
	printf("\r\n");
}


static void PD_Print_ExtPayload_Indexed(void)
{
	u8 i;
	u8 size = rxExtHeader->DataSize;
	u8 *payload = ((u8 *)PD_RX_BUF) + 4;
	if ( size > 48 ) {
		size = 48;
	}
	printf("  EXT payload bytes:");
	for ( i = 0; i < size; i++ ) {
		printf(" [%d]=%02X", i, payload[i]);
	}
	printf("\r\n");
}


static void PD_Decode_Status_Message(void)
{
	u8 *p = ((u8 *)PD_RX_BUF) + 4;
	printf("  Status decode: internalTempRaw=0x%02X presentInputRaw=0x%02X eventFlags=%02X %02X %02X %02X %02X\r\n",
		p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
	printf("  Status note: field names are tentative; raw payload bytes above are authoritative.\r\n");
	printf("@PD1,type=source_status,session=%u,size=%u,raw=%02X%02X%02X%02X%02X%02X%02X\r\n",
	       (unsigned)s_pd_result.session, (unsigned)rxExtHeader->DataSize,
	       p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
}

static void PD_Decode_PPS_Status_Message(void)
{
	u8 *p = ((u8 *)PD_RX_BUF) + 4;
	u16 out_mv = (u16)((p[0] | ((u16)p[1] << 8)) * 20);
	u16 out_ma = (u16)(p[2] * 50);
	printf("  PPS Status decode: output=%umV current=%umA flags=0x%02X\r\n", out_mv, out_ma, p[3]);
	printf("  PPS Status note: valid mainly after PPS contract; under Fixed contract some supplies return placeholder/status bytes.\r\n");
	printf("@PD1,type=pps_status,session=%u,output_mv=%u,current_ma=%u,flags=%02X\r\n",
	       (unsigned)s_pd_result.session, (unsigned)out_mv, (unsigned)out_ma,
	       (unsigned)p[3]);
}

/* Source_Info Data Object decode (PD 3.1 sec 6.5.1 Get_Source_Info response).
 * bits[7:0]=PDP(1W), bits[15:8]=Temperature(°C 0xFF=N/A), bits[25:24]=PresentInput */
static void PD_Decode_Source_Info(void)
{
	/* Source_Info Data Object layout (USB PD 3.1 Table 6-65):
	 *   bits[7:0]   = Port Power Delivery Capable (PDP), 1W LSB
	 *   bits[15:8]  = Internal Temp (0xFF=N/A, 0xFE=throttling; raw °C otherwise)
	 *   bits[17:16] = Present Input (0=None, 1=AC, 2=DC, 3=AC+DC)
	 * NOTE: Some firmware returns placeholder data — all fields marked tentative. */
	static const char *input_str[] = { "None", "AC Main", "DC", "AC+DC" };
	u32 do0  = ((u32)PD_RX_BUF[2] << 16) | PD_RX_BUF[1];
	u8  pdp  = (u8)(do0 & 0xFFu);
	u8  temp = (u8)((do0 >> 8) & 0xFFu);
	u8  inp  = (u8)((do0 >> 16) & 0x03u);
	printf("  Source_Info (tentative): PDP=%uW", (unsigned)pdp);
	if ( temp == 0xFF ) {
		printf(" Temp=N/A");
	} else if ( temp == 0xFE ) {
		printf(" Temp=throttling");
	} else if ( temp >= 100u ) {
		/* > 100°C is unrealistic for a charger under normal conditions;
		 * likely a placeholder or uninitialized value. */
		printf(" TempRaw=0x%02X (%u\xB0""C, not realistic)", (unsigned)temp, (unsigned)temp);
	} else {
		printf(" Temp=%u\xB0""C", (unsigned)temp);
	}
	printf(" PresentInput=%s raw:0x%08lX\r\n", input_str[inp], (unsigned long)do0);
	printf("@PD1,type=source_info,session=%u,pdp_w=%u,temp_raw=%u,present_input=%u,raw=%08lX\r\n",
	       (unsigned)s_pd_result.session, (unsigned)pdp, (unsigned)temp,
	       (unsigned)inp, (unsigned long)do0);
}

/*
 * Revision Data Object デコード。
 * 実測ログの一致からバイト配置を帰納:
 *   payload[3] (MSB) = BCD 形式の PD Spec Revision: 上位ニブル=Major, 下位ニブル=Minor
 *                      例: 0x31 → Rev 3.1, 0x32 → Rev 3.2
 *   payload[2]       = BCD 形式の USB Version: 例: 0x10 → Ver 1.0, 0x17 → Ver 1.7
 *   payload[1:0]     = reserved (must be 0x0000)
 *
 * Cypress(KFD) 0x31 0x10 → PD Rev 3.1 / USB Ver 1.0  ← 信頼性高
 * HKY det      0x32 0x10 → PD Rev 3.2 / USB Ver 1.0
 * Delta/HKY    0x31 0x17 → PD Rev 3.1 / USB Ver 1.7
 * Phihong/Ugreen 0x31 0x18 → PD Rev 3.1 / USB Ver 1.8
 * AOHI 240W    0x32 0x11 → PD Rev 3.2 / USB Ver 1.1
 */
static void PD_Decode_Revision_Message(void)
{
	u8 *payload = ((u8 *)PD_RX_BUF) + 2;
	u8 rev_bcd  = payload[3];
	u8 ver_bcd  = payload[2];
	u8 rev_maj  = (rev_bcd >> 4) & 0xF;
	u8 rev_min  = rev_bcd & 0xF;
	u8 ver_maj  = (ver_bcd >> 4) & 0xF;
	u8 ver_min  = ver_bcd & 0xF;
	printf("  Revision: USB PD Rev %u.%u  (USB Ver %u.%u)\r\n",
		rev_maj, rev_min, ver_maj, ver_min);
	printf("@PD1,type=revision,session=%u,pd_major=%u,pd_minor=%u,usb_major=%u,usb_minor=%u,raw=%08lX\r\n",
	       (unsigned)s_pd_result.session, (unsigned)rev_maj, (unsigned)rev_min,
	       (unsigned)ver_maj, (unsigned)ver_min,
	       (unsigned long)(((u32)PD_RX_BUF[2] << 16) | PD_RX_BUF[1]));
}

static void PD_Decode_Source_Cap_Extended(void)
{
	u8 *p = ((u8 *)PD_RX_BUF) + 4;
	u8 size = rxExtHeader->DataSize;
	if ( size < 24u ) {
		printf("  SourceCapExt: too short (%u bytes)\r\n", (unsigned)size);
		printf("@PD1,type=source_cap_ext,session=%u,size=%u,decode=truncated\r\n",
		       (unsigned)s_pd_result.session, (unsigned)size);
		PD_Print_ExtPayload_Indexed();
		return;
	}
	u16 vid = (u16)p[0] | ((u16)p[1] << 8);
	u16 pid = (u16)p[2] | ((u16)p[3] << 8);
	u32 xid = (u32)p[4] | ((u32)p[5] << 8) | ((u32)p[6] << 16) | ((u32)p[7] << 24);
	printf("  SourceCapExt: VID=0x%04X PID=0x%04X XID=0x%08lX FW=%u HW=%u SPR_PDP=%uW",
	       vid, pid, (unsigned long)xid, (unsigned)p[8], (unsigned)p[9],
	       (unsigned)p[23]);
	if ( size >= 25u ) printf(" EPR_PDP=%uW", (unsigned)p[24]);
	printf("\r\n");
	printf("@PD1,type=source_cap_ext,session=%u,vid=%04X,pid=%04X,xid=%08lX,size=%u,fw_version=%u,hw_version=%u,voltage_regulation_raw=%u,hold_up_time_raw=%u,compliance_raw=%u,touch_current_raw=%u,peak_current_1_raw=%u,peak_current_2_raw=%u,peak_current_3_raw=%u,touch_temperature_raw=%u,source_inputs_raw=%u,fixed_batteries=%u,swappable_batteries=%u,spr_pdp_w=%u,epr_pdp_w=%u\r\n",
	       (unsigned)s_pd_result.session, vid, pid, (unsigned long)xid,
	       (unsigned)size, (unsigned)p[8], (unsigned)p[9],
	       (unsigned)p[10], (unsigned)p[11], (unsigned)p[12], (unsigned)p[13],
	       (unsigned)((u16)p[14] | ((u16)p[15] << 8)),
	       (unsigned)((u16)p[16] | ((u16)p[17] << 8)),
	       (unsigned)((u16)p[18] | ((u16)p[19] << 8)),
	       (unsigned)p[20], (unsigned)p[21], (unsigned)(p[22] & 0x0Fu),
	       (unsigned)((p[22] >> 4) & 0x0Fu), (unsigned)p[23],
	       (unsigned)((size >= 25u) ? p[24] : 0u));
	PD_Print_ExtPayload_Indexed();
}
static void PD_Decode_Manufacturer_Info(void)
{
	u8 *p = ((u8 *)PD_RX_BUF) + 4;
	u8 size = rxExtHeader->DataSize;
	u8 i;
	u16 vid, pid;
	char safe_name[33];
	u8 safe_len = 0;
	safe_name[0] = 0;
	if ( size < 4 ) {
		printf("  MfgInfo: too short (%d bytes)\r\n", size);
		return;
	}
	vid = (u16)p[0] | ((u16)p[1] << 8);
	pid = (u16)p[2] | ((u16)p[3] << 8);
	printf("  MfgInfo: VID=0x%04X PID=0x%04X", vid, pid);
	if ( size > 4 ) {
		u8 name_len = size - 4;
		u8 effective_len = 0;
		/* 末尾の null / スペースを除いた実効長を求める */
		for ( i = 0; i < name_len && i < 32; i++ ) {
			char c = (char)p[4 + i];
			if ( c == 0 ) break;
			if ( (c >= 0x20 && c < 0x7F) && c != ' ' ) effective_len = i + 1;
		}
		if ( effective_len > 0 ) {
			printf(" Name=\"");
			for ( i = 0; i < effective_len; i++ ) {
				char c = (char)p[4 + i];
				printf("%c", (c >= 0x20 && c < 0x7F) ? c : '?');
				if ( safe_len < 32u ) {
					if ( (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
					     (c >= 'a' && c <= 'z') || c == '-' || c == '.' ) {
						safe_name[safe_len++] = c;
					} else {
						safe_name[safe_len++] = '_';
					}
				}
			}
			printf("\"");
		}
		else {
			printf(" Name=(empty/spaces only)");
		}
	}
	safe_name[safe_len] = 0;
	printf("\r\n");
	printf("@PD1,type=manufacturer,session=%u,vid=%04X,pid=%04X,name=%s\r\n",
	       (unsigned)s_pd_result.session, vid, pid,
	       safe_len ? safe_name : "unavailable");
}

static void PD_Decode_BatteryCap(void)
{
	u8 *p = ((u8 *)PD_RX_BUF) + 4;
	u8 size = rxExtHeader->DataSize;
	printf("  BatteryCap: DataSize=%d bytes\r\n", size);
	if ( size >= 6 ) {
		u16 vid       = (u16)p[0] | ((u16)p[1] << 8);
		u16 pid       = (u16)p[2] | ((u16)p[3] << 8);
		u16 design    = (u16)p[4] | ((u16)p[5] << 8);
		printf("  BatteryCap: VID=0x%04X PID=0x%04X DesignCap=%u×0.1Wh", vid, pid, design);
		if ( size >= 8 ) {
			u16 last_full = (u16)p[6] | ((u16)p[7] << 8);
			printf(" LastFullCap=%u×0.1Wh", last_full);
		}
		if ( size >= 9 ) {
			printf(" BattType=0x%02X", p[8]);
		}
		printf("\r\n");
		printf("@PD1,type=battery_cap,session=%u,vid=%04X,pid=%04X,design_01wh=%u,last_full_01wh=%u,battery_type=%u\r\n",
		       (unsigned)s_pd_result.session, vid, pid, (unsigned)design,
		       (unsigned)(size >= 8 ? ((u16)p[6] | ((u16)p[7] << 8)) : 0u),
		       (unsigned)(size >= 9 ? p[8] : 0u));
	}
}

static void PD_Decode_BatteryStatusData(void)
{
	u32 dw = ((u32)PD_RX_BUF[2] << 16) | PD_RX_BUF[1];
	u8  flags   = (u8)(dw & 0x0F);
	u16 cap_raw = (u16)((dw >> 4) & 0xFFFFF);
	printf("  BatteryStatus raw:0x%08lX flags:0x%X capacity_raw:%u\r\n",
		(unsigned long)dw, flags, cap_raw);
	printf("  BatteryStatus flags: InvalidBatRef=%d BattPresent=%d BattIsCharging=%d BattIsDischarg=%d\r\n",
		(flags >> 0) & 1, (flags >> 1) & 1, (flags >> 2) & 1, (flags >> 3) & 1);
	printf("@PD1,type=battery_status,session=%u,raw=%08lX,flags=%u,capacity_raw=%u\r\n",
	       (unsigned)s_pd_result.session, (unsigned long)dw,
	       (unsigned)flags, (unsigned)cap_raw);
}

static void PD_Decode_Country_Codes(void)
{
	u8 *p = ((u8 *)PD_RX_BUF) + 4;
	u8 size = rxExtHeader->DataSize;
	printf("@PD1,type=country_codes,session=%u,size=%u,first=%02X%02X%02X%02X\r\n",
	       (unsigned)s_pd_result.session, (unsigned)size,
	       size > 0u ? p[0] : 0u, size > 1u ? p[1] : 0u,
	       size > 2u ? p[2] : 0u, size > 3u ? p[3] : 0u);
}

static void PD_InfoProbe_LogResponse(void)
{
	printf("RX probe response for %s: ", PD_InfoProbe_Name(PD_InfoProbe_Step));
	if ( rxHeader->Extended ) {
		printf("Extended %s MsgType:0x%02X NDO:%d Size:%d Chunked:%d Chunk:%d\r\n",
			PD_Ext_Msg_Name(rxHeader->MsgType), rxHeader->MsgType, rxHeader->NDO,
			rxExtHeader->DataSize, rxExtHeader->Chunked, rxExtHeader->ChunkNumber);
		PD_Print_Raw_RX("  EXT");
		if ( rxHeader->MsgType == PD_Ext_SrcCapExtended ) {
			PD_Decode_Source_Cap_Extended();
		}
		else if ( rxHeader->MsgType == PD_Ext_Status ) {
			PD_Print_ExtPayload_Indexed();
			PD_Decode_Status_Message();
		}
		else if ( rxHeader->MsgType == PD_Ext_PPSStatus ) {
			PD_Print_ExtPayload_Indexed();
			PD_Decode_PPS_Status_Message();
		}
		else if ( rxHeader->MsgType == PD_Ext_ManufacturerInfo ) {
			PD_Decode_Manufacturer_Info();
		}
		else if ( rxHeader->MsgType == PD_Ext_BattertCap ) {
			PD_Decode_BatteryCap();
		}
		else if ( rxHeader->MsgType == PD_Ext_CountryCodes ) {
			PD_Print_ExtPayload_Indexed();
			PD_Decode_Country_Codes();
		}
		else {
			PD_Print_ExtPayload_Indexed();
		}
	}
	else if ( rxHeader->NDO ) {
		printf("Data %s MsgType:0x%02X NDO:%d\r\n", PD_Data_Msg_Name(rxHeader->MsgType), rxHeader->MsgType, rxHeader->NDO);
		PD_Print_Raw_RX("  DATA");
		if ( rxHeader->MsgType == PD_Data_Revision && rxHeader->NDO >= 1 ) {
			PD_Decode_Revision_Message();
		}
		else if ( rxHeader->MsgType == PD_Data_SrcInfo && rxHeader->NDO >= 1 ) {
			PD_Decode_Source_Info();
		}
		else if ( rxHeader->MsgType == PD_Data_BatteryStatus && rxHeader->NDO >= 1 ) {
			PD_Decode_BatteryStatusData();
		}
	}
	else {
		printf("Control %s MsgType:0x%02X\r\n", PD_Ctrl_Msg_Name(rxHeader->MsgType), rxHeader->MsgType);
	}
}

/*
 * Extended メッセージ(Get_Manufacturer_Info / Get_Battery_Cap / Get_Battery_Status)を
 * プローブとして送信する。
 * PD_PHY_Header_Init で設定したヘッダの Extended ビットを手動で立て、
 * Extended Header と ペイロードをセットしてから PD_Prot_pSet で送信キューに入れる。
 */
static void PD_InfoProbe_SendExt(u8 ext_type, u8 data_size, u8 d0, u8 d1)
{
	st_Extended_Header *txExtHdr;
	u8 *payload;

	/* NDO=1: Extended Header(2B) + Data(≤2B) = 4B = 1×32bit object */
	PD_PHY_Header_Init(1, 1, ext_type);
	txHeader->Extended = 1;

	txExtHdr = (st_Extended_Header *)&PD_TX_BUF[1];
	txExtHdr->Data        = 0;
	txExtHdr->DataSize    = data_size;

	payload    = ((u8 *)PD_TX_BUF) + 4;
	payload[0] = d0;
	payload[1] = d1;

	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 450;   /* tSenderResponse 相当 (~30ms) に拡大 */
	PD_Prot_pSet( NULL , PD_InfoProbe_RX , PD_InfoProbe_TxFailed , PD_InfoProbe_RxTimeout );
}

static void PD_InfoProbe_Start(void)
{
#if !PD_ANALYZER_INFO_PROBES
	PD_InfoProbe_Active = 0;
	PD_InfoProbe_Step = 0;
	PD_InfoProbe_Done = 1;
	PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
	printf("Info probes disabled; try minimal EPR path\r\n");
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_EPR_Enter_Probe_If_Capable();
	return;
#endif
	if ( PD_InfoProbe_Active ) {
		return;
	}
	if ( PD_InfoProbe_Done ) {
		printf("Info probes already done; skip GET_* and try EPR path\r\n");
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		PD_EPR_Enter_Probe_If_Capable();
		return;
	}
	PD_InfoProbe_Active = 1;
	PD_InfoProbe_TxRetry = 0;
	PD_InfoProbe_NoResponseCnt = 0;
	/*
	 * Sink_Capabilities はプローブ前ではなく EPR_Mode Enter 直前に送信する。
	 * プローブ前に送ると Anker/KFD 等が Soft_Reset を発してプローブが妨害される。
	 * s_epr_sinkcap_sent を 0 に戻して、新しいプローブサイクルで再送を許可する。
	 */
	s_epr_sinkcap_sent = 0;
	PD_InfoProbe_Step = 1;
	PD_InfoProbe_SendNext();
}

static void PD_InfoProbe_Finish(void)
{
	PD_InfoProbe_Active = 0;
	PD_InfoProbe_Step = 0;
	PD_InfoProbe_TxRetry = 0;
	PD_InfoProbe_NoResponseCnt = 0;
	PD_InfoProbe_Done = 1;
	/*
	 * プローブ試行済みフィンガープリントを記録。
	 * 以降に同一ソースが再接続しても再プローブしない（無限ループ防止）。
	 */
	PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	printf("Info probes complete; try EPR path\r\n");
	PD_EPR_Enter_Probe_If_Capable();
}

/* ===================================================================
 * EPR Mode Probe (PD 3.1 / 3.2 Extended Power Range)
 *
 * シーケンス:
 *   EPR_Mode Enter (DO[0]=0x0010) →
 *   EPR_Mode Acknowledged (DO[0]=0x0020) →
 *   EPR_Source_Capabilities (Ext Type=0x11) → decode EPR PDOs →
 *   EPR_Request (RDO + requested Source PDO copy) → Accept → PS_RDY →
 *   EPR_Mode Exit (Action=0x05)
 * =================================================================== */

/* EPR_Source_Capabilities reassembly state (up to Object 11). */
static u32 s_epr_pdo_buf[11];
static u16 s_epr_total_size;
static u8  s_epr_obj7_partial[2];
static u8  s_epr_softreset_wait_cnt;
static u8  s_epr_chunk_request_retry;
static u8  s_epr_exit_async_rx_count;

enum {
	PD_EPR_REQUEST_FIXED = 0,
	PD_EPR_REQUEST_OBJ5_PPS,
	PD_EPR_REQUEST_RESTORE_FIXED
};

static u8  s_epr_request_stage;
static u16 s_epr_pps_req_mv;

static void PD_EPR_Print_Reconstructed(void);
static void PD_EPR_Exit(void);
static void PD_EPR_SendRequest(void);
static void PD_EPR_SendRequestPacket(u8 obj, u32 rdo, u32 pdo,
	                                 const char *purpose);

static void PD_EPR_Exit_NoResponse(void)
{
	PD_PHY.WaitMsgRx = 0;
	s_epr_exit_async_rx_count = 0;
	s_pd_result.epr_exit_status = PD_RESULT_EXIT_TIMEOUT;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	printf("\r\nEPR Mode Exit: no response; assuming returned to SPR\r\n");
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_Source_VDM_Probe_Arm_Delayed(250);
}

static void PD_EPR_Exit_RX(void)
{
	st_VDM_Header *vdm = (st_VDM_Header *)&PD_RX_BUF[1];

	PD_PHY.WaitMsgRx = 0;
	printf("\r\n");
	if ( rxHeader->Extended == 0 && rxHeader->NDO == 1 &&
	     rxHeader->MsgType == PD_Data_EPRMode ) {
		u32 do0    = ((u32)PD_RX_BUF[2] << 16) | PD_RX_BUF[1];
		u8  action = (u8)(do0 >> 24);
		if ( action == 0 ) action = (u8)((do0 >> 4) & 0x0F);

		if ( action == 5 ) {  /* Exit Acknowledged — PD 3.1 Table 6-38 */
			s_pd_result.epr_exit_status = PD_RESULT_EXIT_ACK;
			printf("EPR Mode: Exit Acknowledged — back to SPR\r\n");
		} else {
			s_pd_result.epr_exit_status = PD_RESULT_EXIT_UNEXPECTED;
			printf("EPR Mode Exit: action=0x%X\r\n", (unsigned)action);
		}
	} else {
		if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
		     rxHeader->MsgType == PD_Ctrl_SoftReset ) {
			s_pd_result.epr_exit_status = PD_RESULT_EXIT_SOFT_RESET;
			printf("EPR Mode Exit: Source issued Soft_Reset; treating as returned to SPR\r\n");
			/* SoftReset に正しく応答する — pProt_RX_SoftRst は後続の
			 * pProt_IDLE 設定で上書きされるため、ここでは単純に SPR 復帰とみなす。 */
		} else if ( !rxHeader->Extended && rxHeader->NDO != 0 &&
		            rxHeader->MsgType == PD_Data_SrcCap ) {
			s_epr_exit_async_rx_count = 0;
			s_pd_result.epr_exit_status = PD_RESULT_EXIT_SOURCE_CAP;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			printf("EPR Mode Exit: Source sent Source_Capabilities; re-establish SPR contract before discovery\r\n");
			/*
			 * This Source_Capabilities is the start of the post-Exit SPR
			 * negotiation, not merely an Exit acknowledgement.  Consume the
			 * packet now so Request is sent inside tSenderResponse.  Starting
			 * Discover Identity first makes LA280PM240 issue Hard Reset.
			 */
			PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
			pProt_RX_SrcCap();
			return;
		} else {
			/*
			 * Source-initiated VDMs and other asynchronous traffic are not
			 * responses to EPR_Mode Exit.  The PHY has already returned
			 * GoodCRC; keep the Exit transaction alive until Source_Cap,
			 * Soft_Reset, or the timeout arrives.
			 */
			s_epr_exit_async_rx_count++;
			if ( PD_PHY.LastRxSop == PD_PHY_RX_SOP &&
			     !rxHeader->Extended && rxHeader->NDO > 0u &&
			     rxHeader->MsgType == PD_Data_VendorDefined &&
			     vdm->Type && vdm->CommandType == 0u &&
			     vdm->Command == PD_VDM_DiscoverSVIDs &&
			     vdm->SVID == 0xFF00u ) {
				/*
				 * LA280PM240 puts Source_Capabilities and Discover SVIDs
				 * back-to-back after Exit.  It Hard Resets after any VDM
				 * response from this analyzer, so leave only the hardware
				 * GoodCRC and resume SPR negotiation after the source AMS.
				 */
				printf("EPR Mode Exit: Discover SVIDs arrived after missed "
				       "Source_Capabilities; GoodCRC only and defer SPR Request\r\n");
				s_epr_exit_async_rx_count = 0;
				s_pd_result.epr_exit_status = PD_RESULT_EXIT_SOURCE_CAP;
				s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
				VDM_State.Explicit_Contract_Established = 0;
				PD_PHY.WaitMsgRx = 0u;
				PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
				PD_PostExit_Request_DelayMs = 15u;
				return;
			}
			printf("EPR Mode Exit: asynchronous message 0x%02X (NDO=%d Extended=%d); "
			       "GoodCRC only, keep waiting for Source_Capabilities\r\n",
			       rxHeader->MsgType, rxHeader->NDO, rxHeader->Extended);
			if ( s_epr_exit_async_rx_count <= 8u ) {
				PD_PHY.WaitMsgRx = 1;
				PD_PHY.MsgRxCnt = 1000;
				PD_Prot_pSet( NULL , PD_EPR_Exit_RX , NULL ,
				              PD_EPR_Exit_NoResponse );
				return;
			}
			printf("EPR Mode Exit: too many asynchronous messages\r\n");
			PD_EPR_Exit_NoResponse();
			return;
		}
	}
	s_epr_exit_async_rx_count = 0;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_Source_VDM_Probe_Arm_Delayed(250);
}

static void PD_EPR_Exit(void)
{
	printf("TX EPR_Mode Exit\r\n");
	s_epr_exit_async_rx_count = 0;
	PD_TX_BUF[1] = 0x0000;
	PD_TX_BUF[2] = 0x0500;  /* EPRMDO Action=Exit EPR Mode (0x05) in bits[31:24] */
	
	PD_PHY_Header_Init(5, 1, PD_Data_EPRMode);	
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 450;
	PD_Prot_pSet( NULL , PD_EPR_Exit_RX , NULL , PD_EPR_Exit_NoResponse );
}

static void PD_EPR_Request_Failed(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( s_epr_request_stage == PD_EPR_REQUEST_OBJ5_PPS ) {
		s_pd_result.pps_probe_status = PD_RESULT_STATUS_FAIL;
		s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
		printf("\r\nEPR Object 5 PPS Request failed; original 5V contract remains active\r\n");
	} else if ( s_epr_request_stage == PD_EPR_REQUEST_RESTORE_FIXED ) {
		printf("\r\nEPR fixed-contract restore failed\r\n");
	}
	printf("\r\nEPR Request failed; exit EPR Mode\r\n");
	PD_EPR_Exit();
}

static void PD_EPR_Request_PS_RDY_RX(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0u &&
	     rxHeader->MsgType == PD_Ctrl_PS_Ready ) {
		if ( s_epr_request_stage == PD_EPR_REQUEST_OBJ5_PPS ) {
			u8 obj = PD_Stored_Request_ObjectPos;
			u32 rdo = PD_Stored_Request_RDO;
			u32 pdo;

			s_pd_result.pps_probe_status = PD_RESULT_STATUS_PASS;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			printf("\r\nRX Accept + PS_RDY: EPR Object 5 PPS contract active "
			       "(%umV/%umA)\r\n",
			       (unsigned)s_epr_pps_req_mv,
			       (unsigned)PD_ANALYZER_EPR_PPS_CURRENT_MA);

			if ( obj == 0u || obj > 7u ) obj = 1u;
			pdo = s_epr_pdo_buf[obj - 1u];
			if ( pdo == 0u ) pdo = PD_Stored_Selected_PDO;
			if ( pdo == 0u || rdo == 0u ) {
				printf("EPR Request: missing fixed RDO/PDO for restore\r\n");
				PD_EPR_Exit();
				return;
			}
			s_epr_request_stage = PD_EPR_REQUEST_RESTORE_FIXED;
			PD_EPR_SendRequestPacket(obj, rdo, pdo, "restore fixed 5V");
			return;
		}
		printf("\r\nRX Accept + PS_RDY: 5V EPR contract established\r\n");
		PD_EPR_Print_Reconstructed();
		printf("EPR_Source_Capabilities: complete\r\n");
		PD_EPR_Exit();
		return;
	}
	if ( !rxHeader->Extended && rxHeader->NDO == 0u &&
	     rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
		return;
	}
	PD_EPR_Request_Failed();
}

static void PD_EPR_Request_Accept_RX(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0u &&
	     rxHeader->MsgType == PD_Ctrl_Accept ) {
		PD_PHY.WaitMsgRx = 1u;
		/* LA280PM240 can take about 737 ms from Accept to PS_RDY. */
		PD_PHY.MsgRxCnt  = 1500u;
		PD_Prot_pSet(NULL, PD_EPR_Request_PS_RDY_RX, NULL,
		             PD_EPR_Request_Failed);
		return;
	}
	if ( !rxHeader->Extended && rxHeader->NDO == 0u &&
	     rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
		return;
	}
	PD_EPR_Request_Failed();
}

static void PD_EPR_SendRequestPacket(u8 obj, u32 rdo, u32 pdo,
	                                 const char *purpose)
{
	/*
	 * USB PD 3.x 6.4.8: EPR_Request contains exactly two objects:
	 * the RDO followed by an exact copy of the requested Source PDO.
	 */
	PD_TX_BUF[1] = (u16)(rdo & 0xFFFFu);
	PD_TX_BUF[2] = (u16)(rdo >> 16);
	PD_TX_BUF[3] = (u16)(pdo & 0xFFFFu);
	PD_TX_BUF[4] = (u16)(pdo >> 16);
	PD_PHY_Header_Init(0, 2, PD_Data_EPRRequest);
	PD_PHY.WaitMsgRx = 1u;
	PD_PHY.MsgRxCnt  = 100u;
	PD_Prot_pSet(NULL, PD_EPR_Request_Accept_RX, PD_EPR_Request_Failed,
	             PD_EPR_Request_Failed);
	PD_PHY_FlushTxNow();
	PD_PHY_Poll();
	printf("\r\nTX EPR_Request: RDO=0x%08lX PDO%u=0x%08lX (%s)\r\n",
	       (unsigned long)rdo, (unsigned)obj, (unsigned long)pdo, purpose);
}

static void PD_EPR_SendRequest(void)
{
	u8 obj = PD_Stored_Request_ObjectPos;
	u32 rdo = PD_Stored_Request_RDO;
	u32 pdo;

#if PD_ANALYZER_EPR_OBJ5_PPS_PROBE
	/*
	 * Some Dell sources advertise a PPS APDO only in the reconstructed EPR
	 * Source Capabilities list.  Probe Object 5 at its minimum voltage and a
	 * low current, then restore the original fixed contract before EPR Exit.
	 */
	if ( s_epr_total_size >= 20u ) {
		u32 pps_pdo = s_epr_pdo_buf[4];
		if ( (pps_pdo >> 30) == 3u &&
		     ((pps_pdo >> 28) & 0x03u) == 0u ) {
			u16 min_mv = (u16)(((pps_pdo >> 8) & 0xFFu) * 100u);
			u16 max_mv = (u16)(((pps_pdo >> 17) & 0xFFu) * 100u);
			u16 max_ma = (u16)((pps_pdo & 0x7Fu) * 50u);
			u16 req_mv = (u16)PD_ANALYZER_EPR_PPS_TARGET_MV;
			u16 req_ma = (u16)PD_ANALYZER_EPR_PPS_CURRENT_MA;

			if ( req_mv < min_mv ) req_mv = min_mv;
			if ( req_mv > max_mv ) req_mv = max_mv;
			if ( req_ma > max_ma ) req_ma = max_ma;
			if ( req_mv >= min_mv && req_mv <= max_mv && req_ma >= 50u ) {
				rdo  = (5u << 28);
				rdo |= (1u << 24);  /* No USB Suspend */
				rdo |= (1u << 23);  /* Unchunked Extended supported */
				rdo |= (1u << 22);  /* EPR Mode capable */
				rdo |= ((u32)(req_mv / 20u) << 9);
				rdo |= (u32)(req_ma / 50u);
				s_epr_request_stage = PD_EPR_REQUEST_OBJ5_PPS;
				s_epr_pps_req_mv = req_mv;
				s_pd_result.pps_probe_status = PD_RESULT_STATUS_PENDING;
				s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
				printf("EPR Object 5 is PPS %u-%umV/%umA; probing at "
				       "%umV/%umA\r\n",
				       (unsigned)min_mv, (unsigned)max_mv,
				       (unsigned)max_ma, (unsigned)req_mv,
				       (unsigned)req_ma);
				PD_EPR_SendRequestPacket(5u, rdo, pps_pdo,
				                         "Object 5 PPS probe");
				return;
			}
		}
	}
#endif

	if ( obj == 0u || obj > 7u ) obj = 1u;
	pdo = s_epr_pdo_buf[obj - 1u];
	if ( pdo == 0u ) pdo = PD_Stored_Selected_PDO;
	if ( pdo == 0u || rdo == 0u ) {
		printf("EPR Request: missing current RDO/PDO; exit EPR Mode\r\n");
		PD_EPR_Exit();
		return;
	}
	s_epr_request_stage = PD_EPR_REQUEST_FIXED;
	PD_EPR_SendRequestPacket(obj, rdo, pdo, "keep fixed 5V");
}

/* ------------------------------------------------------------------ */
/* EPR_Source_Capabilities 再構成バッファ                               */
/* chunk 0 / chunk 1 の PDO を蓄積し、全データ取得後に一括表示する。    */
/* ------------------------------------------------------------------ */
static void PD_EPR_Reassembly_Reset(void)
{
	u8 i;
	s_epr_total_size = 0u;
	s_epr_softreset_wait_cnt = 0u;
	s_epr_obj7_partial[0] = 0u;
	s_epr_obj7_partial[1] = 0u;
	s_epr_chunk_request_retry = 0u;
	s_epr_request_stage = PD_EPR_REQUEST_FIXED;
	s_epr_pps_req_mv = 0u;
	for ( i = 0u; i < 11u; i++ ) s_epr_pdo_buf[i] = 0u;
}

/* EPR_Source_Capabilities 再構成テーブルを一括表示 */
static void PD_EPR_Print_Reconstructed(void)
{
	u8 num_objects = (u8)(s_epr_total_size / 4u);
	u8 i;
	if ( num_objects > 11u ) num_objects = 11u;
	printf("EPR_Source_Capabilities reconstructed: %u bytes, %u objects\r\n",
	       (unsigned)s_epr_total_size, (unsigned)num_objects);
	for ( i = 0u; i < num_objects; i++ ) {
		u8          obj    = (u8)(i + 1u);
		u32         pdo    = s_epr_pdo_buf[i];
		const char *region = (obj >= 8u) ? "[EPR]" : "[SPR]";
		if ( pdo == 0u ) {
			printf("  Object %2u : %s reserved/empty\r\n", (unsigned)obj, region);
		} else {
			printf("  Object %2u : %s ", (unsigned)obj, region);
			PD_Print_PDO_Content_Compact(pdo);
			printf("\r\n");
		}
	}
}

static void PD_EPR_SrcCap_RX(void);

static void PD_EPR_SrcCap_Timeout(void)
{
	u8 partial_count;
	PD_PHY.WaitMsgRx = 0;
	if ( s_epr_total_size > 24u && s_epr_chunk_request_retry < 2u ) {
		s_epr_chunk_request_retry++;
		printf("\r\nEPR_Source_Capabilities: chunk 1 timeout; retry request %u/2\r\n",
		       (unsigned)s_epr_chunk_request_retry);
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt  = 600;
		PD_Prot_pSet(NULL, PD_EPR_SrcCap_RX, NULL, PD_EPR_SrcCap_Timeout);
		PD_TX_ChunkRequest();
		PD_PHY_Poll();
		return;
	}
	s_pd_result.chunk_status = PD_RESULT_STATUS_FAIL;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	if ( s_epr_total_size > 0u ) {
		/* chunk 0 は受信済み — 部分データを表示 */
		printf("\r\nEPR_Source_Capabilities: chunk 1 timeout — partial data (chunk 0 only):\r\n");
		PD_EPR_Print_Reconstructed();
		partial_count = (s_epr_total_size < 24u) ? (u8)(s_epr_total_size / 4u) : 6u;
		PD_Result_UpdateEPR(s_epr_pdo_buf, partial_count, 0u);
	} else {
		printf("\r\nEPR_Source_Capabilities: no response from source\r\n");
	}
	/* 念のため Exit を試みる（EPR に入れていないかもしれないが無害） */
	PD_EPR_Exit();
}

static void PD_EPR_SrcCap_RX(void)
{
	PD_PHY.WaitMsgRx = 0;

	/* Do NOT early-return on chunk0 before storing payload — that dropped
	 * chunk0 data and left WaitMsgRx unset (Anker log: chunk0 then idle).
	 * Sniff/printf AFTER Cap handling so Chunk Request is not delayed. */

	if ( rxHeader->Extended && rxHeader->MsgType == PD_Ext_EPRSrcCapabilities ) {
		u8  chunk_num  = rxExtHeader->ChunkNumber;
		u8  is_chunked = rxExtHeader->Chunked;
		u16 total_size = rxExtHeader->DataSize;
		u8 *p = (u8 *)&PD_RX_BUF[2];
		u8  i;

		if ( !is_chunked || chunk_num == 0 ) {
			/*
			 * Unchunked (HKY): store ALL PDOs from DataSize (up to 11).
			 * Chunked chunk0: first 6 (+2B of obj7); request chunk1 if needed.
			 */
			u8 need_chunk1 = (u8)( is_chunked && total_size > 24u );
			u8 num_pdos;

			if ( is_chunked ) {
				num_pdos = (total_size >= 24u) ? 6u : (u8)(total_size / 4u);
			} else {
				num_pdos = (u8)(total_size / 4u);
				if ( num_pdos > 11u ) num_pdos = 11u;
			}

			/* バッファ初期化 */
			s_epr_total_size = total_size;
			s_epr_softreset_wait_cnt = 0;
			s_epr_chunk_request_retry = 0;
			for ( i = 0u; i < 11u; i++ ) s_epr_pdo_buf[i] = 0u;
			s_epr_obj7_partial[0] = 0u;
			s_epr_obj7_partial[1] = 0u;

			/* Objects をバッファへ格納 */
			for ( i = 0u; i < num_pdos; i++ ) {
				s_epr_pdo_buf[i] = (u32)p[0] | ((u32)p[1]<<8) | ((u32)p[2]<<16) | ((u32)p[3]<<24);
				p += 4;
			}

			/* Object 7 の先頭2バイトを保存 (payload byte 24-25) — chunked only */
			if ( need_chunk1 ) {
				s_epr_obj7_partial[0] = p[0];
				s_epr_obj7_partial[1] = p[1];
			}

			if ( need_chunk1 ) {
				/* Arm Cap wait BEFORE Chunk Request so deferred chunk1 has a handler. */
				PD_PHY.WaitMsgRx = 1;
				PD_PHY.MsgRxCnt  = 600;
				PD_Prot_pSet(NULL, PD_EPR_SrcCap_RX, NULL, PD_EPR_SrcCap_Timeout);
				PD_TX_ChunkRequest();
				/* Drain immediately — chunk1 may already be deferred (HKY). */
				PD_PHY_Poll();
				return;
			}

			printf("RX EPR_Source_Capabilities %s (%u bytes, %u objects)\r\n",
			       is_chunked ? "chunk 0" : "unchunked",
			       (unsigned)total_size, (unsigned)num_pdos);

			PD_Result_UpdateEPR(s_epr_pdo_buf, num_pdos, 1u);
			PD_EPR_SendRequest();
		} else {
			/*
			 * Chunk 1: EPR payload の byte 26 以降を処理。
			 * Do not trust chunk0 DataSize alone — aohi-battery advertises
			 * DataSize=32 but delivers 36B (28V+AVS). Use actual NDO payload.
			 */
			u8  payload_len = (u8)((rxHeader->NDO << 2) - 2u);
			u8  off = 0;
			u8  n;

			if ( payload_len >= 2u && s_epr_total_size >= 26u ) {
				s_epr_pdo_buf[6] = (u32)s_epr_obj7_partial[0]
				                 | ((u32)s_epr_obj7_partial[1] << 8)
				                 | ((u32)p[0] << 16)
				                 | ((u32)p[1] << 24);
				off = 2;
			}
			n = (u8)((payload_len - off) / 4u);
			for ( i = 0u; i < n && (7u + i) < 11u; i++ ) {
				u8 *q = &p[off + (i << 2)];
				s_epr_pdo_buf[7u + i] = (u32)q[0] | ((u32)q[1]<<8) | ((u32)q[2]<<16) | ((u32)q[3]<<24);
			}
			s_epr_total_size = (u16)(28u + ((u16)n << 2));
			s_epr_chunk_request_retry = 0;

			printf("RX EPR_Source_Capabilities chunk 1 (%u EPR PDO%s, payload %uB)\r\n",
			       (unsigned)n, n == 1u ? "" : "s", (unsigned)payload_len);
			PD_Result_UpdateEPR(s_epr_pdo_buf, (u8)(s_epr_total_size / 4u), 1u);
			PD_EPR_SendRequest();
		}
	}
	else if ( rxHeader->Extended == 0 && rxHeader->NDO == 1 &&
	          rxHeader->MsgType == PD_Data_EPRMode ) {
		/*
		 * CY4500 実測で判明した双方向ハンドシェイク:
		 *   1. Sink → Source: EPR_Mode Enter (action=1)
		 *   2. Source → Sink: EPR_Mode Acknowledged (action=2)  ← PD_EPR_Mode_Enter_RX 処理済み
		 *   3. Source → Sink: EPR_Mode 双方向要求 (action=1 or action=3)  ← ここ
		 *   4. Source → Sink: EPR_Source_Capabilities (chunk 0 [→ chunk request → chunk 1])
		 *
		 * NOTE: ACK TX (step 4 per PD3.1 spec) は意図的に省略。
		 * CY4500 実測で Delta も KFD も action=3 の GoodCRC だけで EPR_SrcCap を送信しており、
		 * Sink からの ACK を「待っていない」ことが確認されている。
		 * ACK TX を行うと CH211 の TX/RX 排他により EPR_SrcCap の GoodCRC が阻害され、
		 * 特に KFD (chunk 0 が action=3 GoodCRC の 1.27ms 後) でチャンクが取れなかった。
		 * ACK なしで直接 WaitMsgRx を張ることで KFD の chunk 0/1 取得が可能になる。
		 */
		u32 do0    = ((u32)PD_RX_BUF[2] << 16) | PD_RX_BUF[1];
		u8  action = (u8)(do0 >> 24);
		if ( action == 0 ) action = (u8)((do0 >> 4) & 0x0F);
		if ( action == 1 || action == 3 ) {
			s_pd_result.epr_enter_status = PD_RESULT_STATUS_PASS;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			/* Return from the IRQ immediately; KFD sends Cap about 1.27ms later. */
			PD_PHY.WaitMsgRx = 1;
			PD_PHY.MsgRxCnt  = 600;
			PD_Prot_pSet(NULL, PD_EPR_SrcCap_RX, NULL, PD_EPR_SrcCap_Timeout);
		} else if ( action == 2 ) {
			PD_PHY.WaitMsgRx = 1;
			PD_PHY.MsgRxCnt  = 600;
			PD_Prot_pSet(NULL, PD_EPR_SrcCap_RX, NULL, PD_EPR_SrcCap_Timeout);
		} else if ( action == 4 ) {
			s_pd_result.epr_enter_status = PD_RESULT_STATUS_FAIL;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			printf("EPR Mode: Enter Failed (action=4, data=0x%02X); no EPR_Source_Capabilities\r\n",
			       (unsigned)((do0 >> 16) & 0xFFu));
			PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		} else {
			s_pd_result.epr_enter_status = PD_RESULT_STATUS_FAIL;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			printf("EPR Mode: Enter rejected — EPR_Mode action=0x%02X do0=0x%08lX; no EPR_Source_Capabilities\r\n",
			       (unsigned)action, (unsigned long)do0);
			PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		}
	}
	else if ( rxHeader->Extended == 0 && rxHeader->NDO == 0 &&
	          rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		/* KFD uses Soft Reset after a missed EPR Cap GoodCRC, then retransmits
		 * EPR Source Capabilities after our Accept. Keep this receive state and
		 * return from the IRQ without UART output (verified in CY4500 log 086). */
		if ( s_epr_softreset_wait_cnt >= 3u ) {
			s_pd_result.chunk_status = PD_RESULT_STATUS_NO_RESPONSE;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			s_epr_softreset_wait_cnt = 0u;
			pProt_RX_SoftRst();
			return;
		}
		s_epr_softreset_wait_cnt++;
		PD_PHY.TxMsgID = 0u;
		PD_PHY.RxMsgID = 0u;
		txHeader->MsgType      = PD_Ctrl_Accept;
		txHeader->NDO          = 0u;
		txHeader->MsgID        = 0u;
		txHeader->PortDataRole = PD_PHY.Header.PortDataRole;
		txHeader->PortPwrRole  = PD_PHY.Header.PortPwrRole;
		txHeader->SpecRevision = PD_PHY.Header.SpecRevision;
		txHeader->Extended     = 0u;
		PD_PHY.WaitMsgTx = 1u;
		PD_PHY.MsgTxCnt  = 0u;
		PD_PHY_FlushTxNow();
		PD_PHY.WaitMsgRx = 1u;
		PD_PHY.MsgRxCnt  = 600u;
		PD_Prot_pSet(NULL, PD_EPR_SrcCap_RX, NULL, PD_EPR_SrcCap_Timeout);
		return;
	}
	else if ( rxHeader->Extended == 0 && rxHeader->NDO == 0 ) {
		printf("EPR_Source_Capabilities: source replied %s (0x%02X) — EPR mode may have been rejected\r\n",
		       PD_Ctrl_Msg_Name(rxHeader->MsgType), (unsigned)rxHeader->MsgType);
		VDM_Sniff_SOPP_Cable();
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	}
	else {
		printf("EPR_Source_Capabilities: unexpected message (Ext=%d Type=0x%02X NDO=%d); exiting EPR\r\n",
		       rxHeader->Extended, (unsigned)rxHeader->MsgType, rxHeader->NDO);
		VDM_Sniff_SOPP_Cable();
		PD_EPR_Exit();
	}
}

static void PD_EPR_Mode_NoResponse(void)
{
	PD_PHY.WaitMsgRx = 0;
	s_pd_result.epr_enter_status = PD_RESULT_STATUS_NO_RESPONSE;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	printf("\r\nEPR Mode Enter: no response (source timed out)\r\n");
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_Source_VDM_Probe_Arm_Delayed(250);
}

static void PD_EPR_Mode_TxFailed(void)
{
	PD_PHY.WaitMsgRx = 0;
	PD_PHY_Abort();
	s_pd_result.epr_enter_status = PD_RESULT_STATUS_TX_FAILED;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	printf("\r\nEPR Mode Enter: TX failed (no GoodCRC)\r\n");
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_Source_VDM_Probe_Arm_Delayed(250);
}

static void PD_EPR_Mode_Enter_RX(void)
{
	PD_PHY.WaitMsgRx = 0;

	if ( rxHeader->Extended == 0 && rxHeader->NDO == 1 &&
	     rxHeader->MsgType == PD_Data_EPRMode ) {
		u32 do0 = ((u32)PD_RX_BUF[2] << 16) | PD_RX_BUF[1];
		/*
		 * Action フィールド抽出:
		 *   PD 3.1 Table 6-38 仕様: bits[7:4]
		 *   実機観測 (Delta 等): bits[31:24] に格納して送信
		 * bits[31:24] を優先し、ゼロなら bits[7:4] にフォールバック。
		 */
		u8 action = (u8)(do0 >> 24);
		if ( action == 0 ) action = (u8)((do0 >> 4) & 0x0F);

		if ( action == 2 ) {  /* Enter Acknowledged */
			s_pd_result.epr_enter_status = PD_RESULT_STATUS_PENDING;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			/* No UART or VDM work here: the next EPR packet can follow in ~1ms. */
			PD_PHY.WaitMsgRx = 1;
			PD_PHY.MsgRxCnt  = 600;
			PD_Prot_pSet( NULL , PD_EPR_SrcCap_RX , NULL , PD_EPR_SrcCap_Timeout );
			return;
		}
		if ( action == 3 ) {  /* Enter Succeeded */
			s_pd_result.epr_enter_status = PD_RESULT_STATUS_PASS;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			PD_PHY.WaitMsgRx = 1;
			PD_PHY.MsgRxCnt  = 600;
			PD_Prot_pSet( NULL , PD_EPR_SrcCap_RX , NULL , PD_EPR_SrcCap_Timeout );
			return;
		} else if ( action == 4 ) {
			s_pd_result.epr_enter_status = PD_RESULT_STATUS_FAIL;
			s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
			printf("\r\nEPR Mode: Enter Failed (action=4, data=0x%02X)\r\n",
			       (unsigned)((do0 >> 16) & 0xFFu));
		} else {
			printf("\r\nEPR Mode Enter: EPR_Mode action=0x%02X do0=0x%08lX\r\n",
			       (unsigned)action, (unsigned long)do0);
		}
	}
	VDM_Sniff_SOPP_Cable();
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_Source_VDM_Probe_Arm_Delayed(250);
}

/* ===================================================================
 * PPS Contract Probe
 * VDM 完了後、EPR プローブ前に PPS 契約して Get_PPS_Status を取得する。
 * 取得後は元の Fixed 契約を復元し EPR プローブへ移行する。
 * =================================================================== */

/* PPS プローブ完了 → EPR プローブへ */
static void PD_PPS_Probe_End(void)
{
	PD_EPR_Enter_Probe_If_Capable();
}

/* EPR Enter 直前の Sink_Cap 送信完了 → EPR Enter へ */
static void __attribute__((unused)) PD_EPR_SinkCap_Done(void)
{
	PD_PHY.WaitMsgRx = 0;
	PD_EPR_Enter_Probe_If_Capable();
}

static void __attribute__((unused)) PD_EPR_SinkCap_TxFailed(void)
{
	/* GoodCRC miss does not mean the source ignored SinkCap (bus busy with SOP').
	 * Proceed to Enter — same as settle-timeout path when SoftReset never arrives. */
	PD_PHY.WaitMsgRx = 0;
	printf("Sink_Cap (pre-EPR): TX failed (no GoodCRC); still try EPR Enter\r\n");
	PD_EPR_Enter_Probe_If_Capable();
}

/* EPR Enter 直前の Sink_Cap — RX コールバック
 * Source が Soft_Reset を返してきた場合は pProt_RX_SoftRst() に委譲する。
 * 再契約後に PD_EPR_Enter_Probe_If_Capable() が再び呼ばれ、
 * s_epr_sinkcap_sent=1 なので Sink_Cap を再送せず EPR Enter へ進む。 */
static void __attribute__((unused)) PD_EPR_SinkCap_RX(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
	     rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		printf("Sink_Cap (pre-EPR): Source issued Soft_Reset — re-negotiating, will retry EPR Enter\r\n");
		pProt_RX_SoftRst();
		return;
	}
	if ( !rxHeader->Extended && rxHeader->NDO > 0 &&
	     rxHeader->MsgType == PD_Data_SrcCap ) {
		/*
		 * AOHI 240W initially advertises a single 5V PDO, then upgrades to
		 * its full EPR-capable PDO set after seeing our Sink_Capabilities.
		 */
		pProt_RX_SrcCap();
		return;
	}
	/* その他のメッセージ (Source_Cap 等) → そのまま EPR Enter へ */
	PD_EPR_Enter_Probe_If_Capable();
}

/* タイムアウト/失敗 → ログ出力して続行 */
static void PD_PPS_Probe_Failed(void)
{
	PD_PHY.WaitMsgRx = 0;
	s_pd_result.pps_probe_status = PD_RESULT_STATUS_FAIL;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	printf("PPS probe: aborted — continuing\r\n");
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_PPS_Probe_End();
}

/* Fixed 契約復元後 PS_RDY 受信 */
static void PD_PPS_RxRestorePS_RDY(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_PS_Ready ) {
		printf("\r\nRX PS_RDY: Fixed contract restored\r\n");
	} else {
		printf("\r\nPPS restore: unexpected message 0x%02X\r\n", rxHeader->MsgType);
	}
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_PPS_Probe_End();
}

/* Fixed 契約復元 Accept 受信 */
static void PD_PPS_RxRestoreAccept(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_Accept ) {
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt  = 500;
		PD_Prot_pSet( NULL , PD_PPS_RxRestorePS_RDY , NULL , PD_PPS_Probe_Failed );
	} else if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
	} else {
		printf("\r\nPPS restore: rejected (0x%02X)\r\n", rxHeader->MsgType);
		PD_PPS_Probe_Failed();
	}
}

/* Fixed 契約復元リクエスト送信 */
static void PD_PPS_SendRestoreFixed(void)
{
	st_Request_Fixed *pReq = (st_Request_Fixed *)&PD_TX_BUF[1];
	st_SrcCap_Fixed  *pSrc = (st_SrcCap_Fixed  *)&PD_PHY.rxSrcCap[s_pps_orig_obj - 1u];
	u16 ma = pSrc->MaxCurrent;
	printf("TX Request (restoring Fixed contract, PDO Index:%u)\r\n", (unsigned)s_pps_orig_obj);
	pReq->Data         = 0;
	pReq->ObjectPos    = s_pps_orig_obj;
	pReq->NoUSBSuspend = 1;
	pReq->EPRMode            = ( PD_Stored_SrcCap_EprCap ) ? 1 : 0;
	pReq->UnchunkedExtended  = 1;
	pReq->Current = pReq->MaxCurrent = ma;
	PD_PHY_Header_Init( 0 , 1 , PD_Data_Request );
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 60;
	PD_Prot_pSet( NULL , PD_PPS_RxRestoreAccept , NULL , PD_PPS_Probe_Failed );
	PD_PHY_FlushTxNow();
}

/* Get_PPS_Status (PPS 契約後) のレスポンス受信 */
static void PD_PPS_RxStatus(void)
{
	PD_PHY.WaitMsgRx = 0;
	printf("\r\n");
	if ( rxHeader->Extended && rxHeader->MsgType == PD_Ext_PPSStatus ) {
		u8  *p     = ((u8 *)PD_RX_BUF) + 4;
		u16 out_mv = (u16)((p[0] | ((u16)p[1] << 8)) * 20);
		u16 out_ma = (u16)(p[2] * 50);
		printf("PPS Status (after PPS contract %umV): output=%umV/%umA flags=0x%02X\r\n",
		       (unsigned)s_pps_req_mv, (unsigned)out_mv, (unsigned)out_ma, (unsigned)p[3]);
		s_pd_result.pps_probe_status = PD_RESULT_STATUS_PASS;
	} else if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_NotSupported ) {
		printf("Get_PPS_Status (after PPS contract): Not_Supported\r\n");
		s_pd_result.pps_probe_status = PD_RESULT_STATUS_UNSUPPORTED;
	} else if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
		return;
	} else {
		printf("Get_PPS_Status (after PPS contract): unexpected response 0x%02X\r\n", rxHeader->MsgType);
		s_pd_result.pps_probe_status = PD_RESULT_STATUS_FAIL;
	}
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	PD_PPS_SendRestoreFixed();
}

/* PPS 契約 PS_RDY 受信 → Get_PPS_Status 送信 */
static void PD_PPS_RxPS_RDY(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_PS_Ready ) {
		printf("\r\nRX PS_RDY: PPS contract (%umV/500mA) active\r\n", (unsigned)s_pps_req_mv);
		printf("TX probe: Get_PPS_Status\r\n");
		PD_PHY_Header_Init( 1 , 0 , PD_Ctrl_GetPPSStatus );
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt  = 450;
		PD_Prot_pSet( NULL , PD_PPS_RxStatus , PD_PPS_Probe_Failed , PD_PPS_Probe_Failed );
	} else if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
	} else {
		printf("\r\nPPS probe: unexpected PS_RDY (0x%02X)\r\n", rxHeader->MsgType);
		PD_PPS_Probe_Failed();
	}
}

/* PPS リクエスト Accept 受信 */
static void PD_PPS_RxAccept(void)
{
	PD_PHY.WaitMsgRx = 0;
	if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_Accept ) {
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt  = 500;
		PD_Prot_pSet( NULL , PD_PPS_RxPS_RDY , NULL , PD_PPS_Probe_Failed );
	} else if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
	} else if ( !rxHeader->Extended && rxHeader->NDO > 0 &&
	            rxHeader->MsgType == PD_Data_SrcCap ) {
		/* aohi-240w: re-advertise SrcCap instead of Accepting PPS.
		 * Request Fixed PDO1 FIRST (tSenderResponse), then log. */
		PD_PHY.rxSrcCapCnt = rxHeader->NDO * 4;
		memset(PD_PHY.rxSrcCap, 0, sizeof(PD_PHY.rxSrcCap));
		memcpy(PD_PHY.rxSrcCap, &PD_RX_BUF[1], PD_PHY.rxSrcCapCnt);
		PD_Stored_SrcCap_NDO     = rxHeader->NDO;
		PD_Stored_SrcCap_SpecRev = rxHeader->SpecRevision;
		PD_Stored_SrcCap_MsgID   = rxHeader->MsgID;
		PD_Stored_SrcCap_EprCap  = (u8)((PD_PHY.rxSrcCap[0] >> 23) & 1);
		PD_PHY.Header.SpecRevision = rxHeader->SpecRevision;
		PD_Current_SrcCap_Fingerprint = PD_SourceCap_Fingerprint();
		PD_PPS_Probe_Done = 1;
		pProt_TX_Request();
		printf("\r\nPPS probe: Source_Capabilities during PPS Request — restore Fixed PDO1\r\n");
	} else {
		/* Reject 等 — 失敗ではなく「非対応」として続行 */
		s_pd_result.pps_probe_status = PD_RESULT_STATUS_UNSUPPORTED;
		s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
		printf("\r\nPPS probe: Request not accepted (0x%02X) — continuing\r\n", rxHeader->MsgType);
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		PD_PPS_Probe_End();
	}
}

/* PPS APDO を検索して PPS リクエストを送信する。PPS がなければ 0 を返す。 */
static u8 __attribute__((unused)) PD_PPS_Probe_TryStart(void)
{
	u8  i;
	u8  ndo         = PD_Stored_SrcCap_NDO;
	u8  best_idx    = 0;
	u32 best_max_mv = 0;
	u32 rdo;
	u16 req_mv;
	u8  req_a;

	/* 最大電圧の PPS APDO を探す */
	for ( i = 0; i < ndo; i++ ) {
		u32 pdo = PD_PHY.rxSrcCap[i];
		if ( (pdo >> 30) == 3u && ((pdo >> 28) & 0x03u) == 0u ) {
			u32 max_mv = (u32)(((pdo >> 17) & 0xFFu) * 100u);
			if ( max_mv > best_max_mv ) {
				best_max_mv = max_mv;
				best_idx    = (u8)(i + 1u);
			}
		}
	}
	if ( best_idx == 0u ) {
		s_pd_result.pps_probe_status = PD_RESULT_STATUS_NA;
		s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
		return 0u;  /* PPS APDO なし */
	}
	s_pd_result.pps_probe_status = PD_RESULT_STATUS_PENDING;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;

	s_pps_pdo_idx  = best_idx;
	s_pps_orig_obj = PD_Stored_Request_ObjectPos;

	{
		u32 pdo    = PD_PHY.rxSrcCap[best_idx - 1u];
		u32 min_mv = (u32)(((pdo >> 8)  & 0xFFu) * 100u);
		u32 max_mv = (u32)(((pdo >> 17) & 0xFFu) * 100u);
		/* Known-good analyzer path: probe PPS at 9V, clamped to APDO range. */
		req_mv = (u16)PD_ANALYZER_PPS_TARGET_MV;
		if ( req_mv > (u16)max_mv ) req_mv = (u16)max_mv;
		if ( req_mv < (u16)min_mv ) req_mv = (u16)min_mv;
	}
	s_pps_req_mv = req_mv;
	req_a = 10u;  /* 500mA = 10 × 50mA */

	/* PPS RDO 組み立て (USB PD spec Table 6-13)
	 * bit24 NoUSBSuspend, bit23 Unchunked, bit22 EPR Mode Capable */
	rdo  = ((u32)best_idx << 28);
	rdo |= (1u << 24);                      /* No USB Suspend */
	rdo |= (1u << 23);                      /* Unchunked Ext Supported */
	if ( PD_Stored_SrcCap_EprCap ) {
		rdo |= (1u << 22);                  /* EPR Mode Capable */
	}
	rdo |= ((u32)(req_mv / 20u) << 9);      /* Output Voltage (20mV units) */
	rdo |= req_a;                            /* Operating Current (50mA units) */

	printf("TX Request PPS (PDO Index:%u, %umV/500mA)\r\n",
	       (unsigned)best_idx, (unsigned)req_mv);
	PD_TX_BUF[1] = (u16)(rdo & 0xFFFFu);
	PD_TX_BUF[2] = (u16)((rdo >> 16) & 0xFFFFu);
	PD_PHY_Header_Init( 0 , 1 , PD_Data_Request );
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 60;
	PD_Prot_pSet( NULL , PD_PPS_RxAccept , NULL , PD_PPS_Probe_Failed );
	PD_PHY_FlushTxNow();
	return 1u;
}

void PD_EPR_Enter_Probe_If_Capable(void)
{
#if PD_ANALYZER_PPS_PROBE
	/* ────── PPS Contract Probe (実行前に PPS 契約して Get_PPS_Status を取得) ────── */
	if ( PD_PPS_Probe_Done == 0 ) {
		PD_PPS_Probe_Done = 1;
		if ( PD_PPS_Probe_TryStart() ) return;
	}
#else
	PD_PPS_Probe_Done = 1;
#endif
	/* ────── SOP' Cable Probe (VCONN_Swap → Discover Identity on cable) ────── */
	if ( PD_Cable_Probe_TryStart() ) return;
	/* ────── 以降は既存の EPR Probe ────── */

#if PD_ANALYZER_EPR_PRE_SINKCAP
	/* ── Sink_Capabilities 送信（EPR Enter 直前・一度だけ） ──
	 * PD 3.1 spec 8.3.3.5.6: "Sink shall have sent Sink_Capabilities with EPR APDO"
	 * before entering EPR mode.  Send here (not at probe start) so that devices like
	 * Anker/KFD that issue Soft_Reset in response don't disrupt the probe sequence.
	 *
	 * aohi-240w (082): first Cap is often NDO=1 / EPRCap=0 (cable not ready).
	 * Sending EPR SinkCap against that set → SoftReset storm. Wait for upgrade. */
	if ( PD_Stored_SrcCap_EprCap == 0 ) {
		printf("Sink_Cap/EPR: wait for EPR-capable Source_Capabilities (current EPRCap=0)\r\n");
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		PD_Source_VDM_Probe_Arm_Delayed(750);
		return;
	}
	if ( s_epr_sinkcap_sent == 0 ) {
		s_epr_sinkcap_sent = 1;
		printf("TX Sink_Capabilities (EPR APDO; wait SoftReset or 150ms then Enter)\r\n");
		Delay_Ms(20);
		PD_PHY.TxSop = PD_PHY_TX_SOP;
		PD_PHY_Set_RxSop(PD_PHY_RX_SOP);
		PD_PHY_Set_ListenOnlySopp(0);
		PD_PHY_Header_Init(1, SinkCapCnt, PD_Data_SinkCap);
		memcpy(&PD_TX_BUF[1], SinkCap, SinkCapCnt * 4);
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt  = 150;
		PD_Prot_pSet(NULL, PD_EPR_SinkCap_RX,
		             PD_EPR_SinkCap_TxFailed, PD_EPR_SinkCap_Done);
		/* Flush immediately — deferred TX races SOP' cable sniff and loses GoodCRC. */
		PD_PHY_FlushTxNow();
		return;
	}
#else
	s_epr_sinkcap_sent = 1;
	if ( PD_Stored_SrcCap_EprCap == 0 ) {
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		return;
	}
#endif
	if ( PD_EPR_Probe_Done ) {
		/* この接続では既に EPR プローブ試行済み */
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		PD_Source_VDM_Probe_Arm_Delayed(250);
		return;
	}
	/* EPR_Attempted_FP: 同一物理接続中の同じ PDO セットへの再試行を抑制する。
	 * Soft/Hard Reset 復帰では保持し、真の CC 切断時だけクリアする。 */
	if ( PD_EPR_Attempted_FP != 0
	  && PD_Current_SrcCap_Fingerprint == PD_EPR_Attempted_FP ) {
		printf("EPR: already attempted for this PDO set (FP:0x%08lX), skipping.\r\n",
		       (unsigned long)PD_EPR_Attempted_FP);
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
		PD_Source_VDM_Probe_Arm_Delayed(250);
		return;
	}
	PD_Source_VDM_Probe_Cancel();
	PD_EPR_Reassembly_Reset();
	s_pd_result.epr_enter_status = PD_RESULT_STATUS_PENDING;
	s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;
	PD_EPR_Probe_Done    = 1;
	PD_EPR_Attempted_FP  = PD_Current_SrcCap_Fingerprint;
	printf("TX EPR_Mode Enter (source advertised EPRCap=1)\r\n");
	Delay_Ms(2);
	PD_PHY.TxSop = PD_PHY_TX_SOP;
	PD_PHY_Set_RxSop(PD_PHY_RX_SOP);
	PD_PHY_Set_ListenOnlySopp(0);
	PD_TX_BUF[1] = 0x0000;
	PD_TX_BUF[2] = 0x0100;  /* Action=Enter(1) in bits 31:24 */

	/* Arm wait BEFORE TX — ACK can follow Enter GoodCRC by ~1–2ms (aohi-240w). */
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 450;
	PD_Prot_pSet( NULL , PD_EPR_Mode_Enter_RX , PD_EPR_Mode_TxFailed , PD_EPR_Mode_NoResponse );
	PD_PHY_Header_Init(0, 1, PD_Data_EPRMode);
	PD_PHY_FlushTxNow();
	/* Drain Mode ACK deferred inside DoTxNow's post-TX poll. */
	PD_PHY_Poll();
}

static void PD_InfoProbe_SendNext(void)
{
	u8 msg;

	/* ── SoftReset でスキップされたステップを飛ばす ── */
	while ( PD_InfoProbe_Step <= 9 && ((PD_InfoProbe_Skip_Mask >> PD_InfoProbe_Step) & 1u) ) {
		printf("Probe: step %d (%s) skipped (caused SoftReset on prior cycle)\r\n",
			PD_InfoProbe_Step, PD_InfoProbe_Name(PD_InfoProbe_Step));
		PD_InfoProbe_Step++;
	}

	/* ── Extended メッセージプローブ (ステップ 7-9) ── */
	if ( PD_InfoProbe_Step == 7 ) {
		printf("TX probe: %s\r\n", PD_InfoProbe_Name(7));
		/* Get_Manufacturer_Info: ManufacturerInfoTarget=0(Port), Ref=0 */
		PD_InfoProbe_SendExt(PD_Ext_GetManufacturerInfo, 2, 0x00, 0x00);
		return;
	}
	if ( PD_InfoProbe_Step == 8 ) {
		printf("TX probe: %s\r\n", PD_InfoProbe_Name(8));
		/* Get_Battery_Cap: BatteryCapDataBlockRef=0 */
		PD_InfoProbe_SendExt(PD_Ext_GetBatteryCap, 1, 0x00, 0x00);
		return;
	}
	if ( PD_InfoProbe_Step == 9 ) {
		printf("TX probe: %s\r\n", PD_InfoProbe_Name(9));
		/* Get_Battery_Status: BatteryStatusDataBlockRef=0 */
		PD_InfoProbe_SendExt(PD_Ext_GetBatteryStatus, 1, 0x00, 0x00);
		return;
	}

	/* ── Control メッセージプローブ (ステップ 1-5) ── */
	msg = PD_InfoProbe_CtrlMsg(PD_InfoProbe_Step);
	if ( msg == 0 ) {
		PD_InfoProbe_Finish();
		return;
	}
	printf("TX probe: %s\r\n", PD_InfoProbe_Name(PD_InfoProbe_Step));
	PD_PHY_Header_Init(1,0,msg);
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 450;   /* tSenderResponse 相当 (~30ms) に拡大 */
	PD_Prot_pSet( NULL , PD_InfoProbe_RX , PD_InfoProbe_TxFailed , PD_InfoProbe_RxTimeout );
}

static void PD_InfoProbe_RX(void)
{
	PD_PHY.WaitMsgRx = 0;

	if ( PD_Is_Extended_Chunk0_With_More_Data() &&
	     rxHeader->MsgType == PD_Ext_EPRSrcCapabilities ) {
		printf("Probe paused: received EPR_Source_Capabilities chunk 0; request chunk 1 now\r\n");
		PD_InfoProbe_Active = 0;
		PD_InfoProbe_Step = 0;
		PD_InfoProbe_TxRetry = 0;
		PD_InfoProbe_Done = 1;
		PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
		PD_EPR_SrcCap_RX();
		return;
	}

	/*
	 * A probe response is not simply "the next packet".  Match the SOP target,
	 * message class and response Message Type.  In particular, an unrelated
	 * source VDM after a corrupt Get_Battery_Cap must not complete that probe.
	 */
	if ( !PD_InfoProbe_Response_Matches(PD_InfoProbe_Step) &&
	     !( !rxHeader->Extended && rxHeader->NDO == 0u &&
	        (rxHeader->MsgType == PD_Ctrl_NotSupported ||
	         rxHeader->MsgType == PD_Ctrl_Reject ||
	         rxHeader->MsgType == PD_Ctrl_Wait ||
	         rxHeader->MsgType == PD_Ctrl_SoftReset) ) ) {
		PD_PHY.WaitMsgRx = 1u;
		PD_PHY.MsgRxCnt  = 450u;
		PD_Prot_pSet(NULL, PD_InfoProbe_RX, PD_InfoProbe_TxFailed,
		             PD_InfoProbe_RxTimeout);
		printf("\r\nProbe %s: ignored unrelated SOP%u message "
		       "(Ext=%u Type=0x%02X NDO=%u)\r\n",
		       PD_InfoProbe_Name(PD_InfoProbe_Step),
		       (unsigned)PD_PHY.LastRxSop, (unsigned)rxHeader->Extended,
		       (unsigned)rxHeader->MsgType, (unsigned)rxHeader->NDO);
		return;
	}

	PD_InfoProbe_NoResponseCnt = 0;
	if ( PD_Is_Extended_Chunk0_With_More_Data() ) {
		PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_PASS);
		PD_InfoProbe_LogResponse();
		printf("Probe paused: Extended MsgType 0x%02X has more data; request chunk 1\r\n",
		       rxHeader->MsgType);
		PD_TX_ChunkRequest();
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt = 600;
		PD_Prot_pSet( NULL , PD_InfoProbe_Chunk_RX , PD_InfoProbe_TxFailed ,
		              PD_InfoProbe_Chunk_Timeout );
		return;
	}

	/*
	 * ソースがプローブ要求に対して SoftReset で返した場合 (非準拠だが実際に起きる)。
	 * SoftReset には必ず Accept で応答しないと MsgID がズレて以降の全通信が失敗する。
	 * → このステップをスキップマスクに記録し、再契約後はここを飛ばして続きから再開。
	 */
	if ( !rxHeader->Extended && rxHeader->NDO == 0 && rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		printf("Probe %s: source sent SoftReset; mark probe_done and recover before EPR.\r\n",
			PD_InfoProbe_Name(PD_InfoProbe_Step));
		PD_InfoProbe_Done = 1;
		PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
		PD_InfoProbe_SoftRst_Cnt++;
		PD_InfoProbe_SoftRst_Recovery_5V_Cnt = 3;
		PD_InfoProbe_Active = 0;
		PD_InfoProbe_Step = 0;
		PD_InfoProbe_TxRetry = 0;
		PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_REJECTED);
		VDM_Reset_Disc_State();
		/* SoftReset に Accept で正しく応答し、再契約フローへ移行 */
		pProt_RX_SoftRst();
		return;
	}

	if ( !rxHeader->Extended && rxHeader->NDO == 0 ) {
		if ( rxHeader->MsgType == PD_Ctrl_NotSupported ) {
			PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_UNSUPPORTED);
		} else if ( rxHeader->MsgType == PD_Ctrl_Reject ) {
			PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_REJECTED);
		} else if ( rxHeader->MsgType == PD_Ctrl_Wait ) {
			PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_BUSY);
		} else {
			PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_INVALID);
		}
	} else {
		PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_PASS);
	}
	PD_InfoProbe_LogResponse();
	PD_InfoProbe_Step++;
	PD_InfoProbe_TxRetry = 0;
	/*
	 * inter-message gap: 次プローブ送信前に少し待つ。
	 * 古いファームでは printf の出力コストが自然なギャップになっていたが、
	 * 現行ファームはより高速なため明示的に待機する。
	 * タイムリーな応答が必要なソースとの互換性向上を狙う。
	 */
	Delay_Ms(5);
	PD_InfoProbe_SendNext();
}

static void PD_InfoProbe_Chunk_RX(void)
{
	PD_PHY.WaitMsgRx = 0;
	PD_InfoProbe_NoResponseCnt = 0;
	printf("\r\n");

	if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
	     rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		printf("Probe %s chunk transfer: source sent SoftReset; recover without restarting probes.\r\n",
		       PD_InfoProbe_Name(PD_InfoProbe_Step));
		PD_InfoProbe_Done = 1;
		PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
		PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_REJECTED);
		pProt_RX_SoftRst();
		return;
	}

	if ( rxHeader->Extended && rxExtHeader->Chunked &&
	     rxExtHeader->RequestChunk == 0 && rxExtHeader->ChunkNumber == 1 ) {
		printf("RX probe chunk 1 for %s: Extended %s MsgType:0x%02X Size:%d\r\n",
		       PD_InfoProbe_Name(PD_InfoProbe_Step),
		       PD_Ext_Msg_Name(rxHeader->MsgType), rxHeader->MsgType,
		       rxExtHeader->DataSize);
		PD_Print_Raw_RX("  EXT CHUNK1");
	} else {
		printf("Probe %s: unexpected response while waiting for chunk 1\r\n",
		       PD_InfoProbe_Name(PD_InfoProbe_Step));
		PD_InfoProbe_LogResponse();
	}

	PD_InfoProbe_Step++;
	PD_InfoProbe_TxRetry = 0;
	Delay_Ms(5);
	PD_InfoProbe_SendNext();
}

static void PD_InfoProbe_Chunk_Timeout(void)
{
	PD_PHY.WaitMsgRx = 0;
	printf("\r\nProbe %s: chunk 1 timeout; continue with chunk 0 data\r\n",
	       PD_InfoProbe_Name(PD_InfoProbe_Step));
	PD_InfoProbe_Step++;
	PD_InfoProbe_TxRetry = 0;
	PD_InfoProbe_SendNext();
}

/*
 * TX failed: GoodCRC が返らなかった。
 * ソースがメッセージを認識しないか、PHY 層で受信できなかった可能性が高い。
 */
static void PD_InfoProbe_TxFailed(void)
{
	printf("\r\nProbe %s: TX no GoodCRC (source may not support this message type at PHY level)\r\n",
		PD_InfoProbe_Name(PD_InfoProbe_Step));
	PD_PHY.WaitMsgRx = 0;
	PD_PHY_Abort();
	PD_PHY.WaitMsgTx = 0;
	PD_PHY.MsgTxCnt = 0;
	PD_PHY.pTxFinish = NULL;
	PD_PHY.pTxTimeout = NULL;
	PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_TX_FAILED);
	PD_InfoProbe_Active = 0;
	PD_InfoProbe_Step = 0;
	PD_InfoProbe_TxRetry = 0;
	PD_InfoProbe_NoResponseCnt = 0;
	PD_InfoProbe_Done = 1;
	PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
	VDM_Reset_Disc_State();
	printf("Probe unsupported: TX no GoodCRC; cancel remaining GET_* probes and try EPR path now\r\n");
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
	PD_EPR_Enter_Probe_If_Capable();
}

/*
 * RX timeout: GoodCRC は受信できたが、応答メッセージが返らなかった。
 * ソースはメッセージを受け取ったが、応答しない（PD2.0 挙動 or PD3.0 非準拠の無視）。
 * PD3.0 準拠なら NotSupported を返すはずだが、多くの安価ソースはこれを送らない。
 */
static void PD_InfoProbe_RxTimeout(void)
{
	printf("\r\nProbe %s: no response (GoodCRC received; source ignores silently)\r\n",
		PD_InfoProbe_Name(PD_InfoProbe_Step));
	PD_PHY.WaitMsgRx = 0;
	PD_Result_RecordProbeStatus(PD_InfoProbe_Step, PD_RESULT_STATUS_NO_RESPONSE);
	PD_InfoProbe_NoResponseCnt++;
	if ( PD_InfoProbe_NoResponseCnt >= 2u ) {
		printf("Probe unavailable: two consecutive requests were acknowledged but unanswered; cancel remaining GET_* probes\r\n");
		PD_InfoProbe_Active = 0;
		PD_InfoProbe_Step = 0;
		PD_InfoProbe_TxRetry = 0;
		PD_InfoProbe_NoResponseCnt = 0;
		PD_InfoProbe_Done = 1;
		PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
		PD_Prot_pSet(NULL, pProt_IDLE, NULL, NULL);
		PD_EPR_Enter_Probe_If_Capable();
		return;
	}
	PD_InfoProbe_Step++;
	PD_InfoProbe_TxRetry = 0;
	PD_InfoProbe_SendNext();
}

enum enum_PD_Ctrl {
	enum_PD_Ctrl_Ignore,
	enum_PD_Ctrl_Reserved,
	enum_PD_Ctrl_txSoftReset,

	enum_PD_Ctrl_GetSrcCap,
	enum_PD_Ctrl_GetSinkCap,
	enum_PD_Ctrl_DRSwap,
	enum_PD_Ctrl_PRSwap,
	enum_PD_Ctrl_SoftReset,
	enum_PD_Ctrl_GetSinkCapExt,
	enum_PD_Ctrl_GetSrcInfo,
	enum_PD_Ctrl_GetRevision,
};

void (*Idle_CtrlMsg_Handle[][2])() = {		//Must correspond to the enum
									/* Source */			 /* Sink */
/* enum_PD_Ctrl_Ignore	    	*/	Prot_NULL,				Prot_NULL,
/* enum_PD_Ctrl_Reserved		*/	PD_RX_Reserved,			PD_RX_Reserved,
/* enum_PD_Ctrl_txSoftReset		*/	pProt_TX_SoftRst,		pProt_TX_SoftRst,

/* enum_PD_Ctrl_GetSrcCap		*/	pProt_TX_SrcCap,		pProt_TX_SrcCap,
/* enum_PD_Ctrl_GetSinkCap		*/	pProt_TX_SinkCap,		pProt_TX_SinkCap,
/* enum_PD_Ctrl_DRSwap	    	*/	pProt_RX_DRSwap,		pProt_RX_DRSwap,
/* enum_PD_Ctrl_PRSwap	    	*/	PD_TX_Reject,			PD_TX_Reject,
/* enum_PD_Ctrl_SoftReset		*/	pProt_RX_SoftRst,		pProt_RX_SoftRst,
/* enum_PD_Ctrl_GetSinkCapExt	*/	PD_TX_SinkCapExt,		PD_TX_SinkCapExt,
/* enum_PD_Ctrl_GetSrcInfo		*/	PD_TX_SrcInfo,			PD_RX_Reserved,
/* enum_PD_Ctrl_GetRevision		*/	PD_TX_Revision,			PD_TX_Revision,
};

const u8 enum_Idle_CtrlMsg[] = {
/* PD_Ctrl_Reserved			 	*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GoodCRC			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GotoMin			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_Accept			 	*/		enum_PD_Ctrl_txSoftReset,
/* PD_Ctrl_Reject			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_Ping				 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_PS_Ready			 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GetSrcCap		 	*/		enum_PD_Ctrl_GetSrcCap,
/* PD_Ctrl_GetSinkCap		 	*/		enum_PD_Ctrl_GetSinkCap,
/* PD_Ctrl_DRSwap			 	*/		enum_PD_Ctrl_DRSwap,
/* PD_Ctrl_PRSwap			 	*/		enum_PD_Ctrl_PRSwap,
/* PD_Ctrl_VconnSwap		 	*/		enum_PD_Ctrl_Reserved,			//Choose Reject or NotSupported based on the protocol version
/* PD_Ctrl_Wait				 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_SoftReset		 	*/		enum_PD_Ctrl_SoftReset,
/* PD_Ctrl_DataReset		 	*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_DataResetComplete 	*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_NotSupported			*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GetSrcCapExtended	*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GetStatus			*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_FRSwap				*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GetPPSStatus			*/		enum_PD_Ctrl_Ignore,
/* PD_Ctrl_GetCountryCodes		*/		enum_PD_Ctrl_Reserved,
/* PD_Ctrl_GetSinkCapExtended	*/		enum_PD_Ctrl_GetSinkCapExt,
/* PD_Ctrl_GetSrcInfo			*/		enum_PD_Ctrl_GetSrcInfo,
/* PD_Ctrl_GetRevision			*/		enum_PD_Ctrl_GetRevision,
};

enum enum_PD_Data {
	enum_PD_Data_Ignore,
	enum_PD_Data_Reserved,
	enum_PD_Data_txSoftReset,

	enum_PD_Data_SrcCap,
	enum_PD_Data_Request,
	enum_PD_Data_BIST,
	enum_PD_Data_VendorDefined,
};

void (*Idle_DataMsg_Handle[][2])() = {		//Must correspond to the enum
									/* Source */			 /* Sink */
/* enum_PD_Data_Ignore	    	*/	Prot_NULL,				Prot_NULL,
/* enum_PD_Data_Reserved		*/	PD_RX_Reserved,			PD_RX_Reserved,
/* enum_PD_Data_txSoftReset		*/	pProt_TX_SoftRst,		pProt_TX_SoftRst,

/*enum_PD_Data_SrcCap	    	*/	Prot_NULL,				pProt_RX_SrcCap,
/*enum_PD_Data_Request	    	*/	pProt_RX_Request,		Prot_NULL,
/*enum_PD_Data_BIST	        	*/	PD_TX_BIST,				PD_TX_BIST,
/*enum_PD_Data_VendorDefined	*/	pProt_RX_VDM,			pProt_RX_VDM,
};

const u8 enum_Idle_DataMsg[] = {
/* PD_Data_Reserved1		*/		enum_PD_Data_Reserved,
/* PD_Data_SrcCap			*/		enum_PD_Data_SrcCap,
/* PD_Data_Request			*/		enum_PD_Data_Request,
/* PD_Data_BIST				*/		enum_PD_Data_BIST,
/* PD_Data_SinkCap			*/		enum_PD_Data_Ignore,
/* PD_Data_BatteryStatus	*/		enum_PD_Data_Ignore,
/* PD_Data_Alert			*/		enum_PD_Data_Ignore,
/* PD_Data_GetCountryInfo	*/		enum_PD_Data_Ignore,
/* PD_Data_EnterUSB			*/		enum_PD_Data_Ignore,
/* PD_Data_EPRRequest		*/		enum_PD_Data_Ignore,
/* PD_Data_EPRMode			*/		enum_PD_Data_Ignore,
/* PD_Data_SrcInfo			*/		enum_PD_Data_Ignore,
/* PD_Data_Revision			*/		enum_PD_Data_Ignore,
/* PD_Data_Reserved2		*/		enum_PD_Data_Reserved,
/* PD_Data_Reserved3		*/		enum_PD_Data_Reserved,
/* PD_Data_VendorDefined	*/		enum_PD_Data_VendorDefined,
};

/*********************************************************************
 * @fn      pDevice_Attached
 *
 * @brief   Check device connection.
 *
 * @return  none
 */
void pDevice_Attached(void)
{
	PD_Result_OnAttach();
	PD_PHY_Reset_Value( PD_DEVICE.DevStat );	//Reset PD PHY related counters
	NVIC_EnableIRQ( USBPD_IRQn );				//Enable PD interrupt

	PD_Disable_Discharge;
	PD_DEVICE.ConnectStat = 1;
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;
	Discover_Identity_Sent = 0;
	/*
	 * プローブ実行中 (PD_InfoProbe_Active == 1) に物理切断された場合:
	 * PD_InfoProbe_Finish() が呼ばれないため PD_Probed_SrcCap_Fingerprint が
	 * 更新されず、同一ソースが再接続するたびに無限プローブが発生する。
	 * → 切断時点で現在の指紋を記録し、同一ソース再接続でプローブを抑制する。
	 * NOTE: EPR SoftReset でも pDevice_Attached が呼ばれる場合があるため、
	 * ここではフィンガープリントを無条件クリアしない。
	 */
	if ( PD_InfoProbe_Active && PD_Current_SrcCap_Fingerprint != 0 ) {
		PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
		PD_InfoProbe_Done = 1;
	}
	PD_InfoProbe_Step  = 0;
	PD_InfoProbe_Active = 0;
	PD_InfoProbe_TxRetry = 0;
	/* デバイス接続時に全フラグをリセット (再接続・再試行を確実にするため) */
	PD_Request_Stop_After_Fail  = 0;
	PD_Request_Fail_Count       = 0;
	PD_PostExit_Request_DelayMs = 0;
	PD_Suppressed_SrcCap_Count  = 0;
	PD_Stored_SrcCap_NDO        = 0;
	PD_InfoProbe_Skip_Mask      = 0;
	PD_InfoProbe_SoftRst_Cnt    = 0;
	/* EPR_Probe_Done は pDevice_Unattached でクリア済み。
	 * EPR_Attempted_FP は MCU リセットまで保持して EPR 無限ループを防ぐ。 */
	VDM_Reset_Disc_State();
	PD_Cable_Sniff_Arm_Delayed();

    if ( PD_DEVICE.DevStat ) {			//Src
    	PD_User_Src_DevIn();
    	( DEF_EN_VCONN )?( PD_TX_VCONN_DISC_IDENT() ):( pProt_TX_SrcCap() );

    }
    else {								//Sink
    	PD_User_Snk_DevIn();
    	pProt_Wait_SrcCap();
    }
}

/*********************************************************************
 * @fn      pDevice_Unattached
 *
 * @brief   Check device removal.
 *
 * @return  none
 */
void pDevice_Unattached(void)
{
	NVIC_DisableIRQ( USBPD_IRQn );			//Turn off PD interrupt

	PD_PHY.WaitTxGcrc = PD_PHY.WaitRxGcrc = PD_PHY.WaitMsgTx = PD_PHY.WaitMsgRx = 0;
#if PD_USE_ANALYZER
	PD_PHY_Abort();
#endif
	PD_DEVICE.ConnectStat = 0;
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;
	Discover_Identity_Sent = 0;
	PD_InfoProbe_Step = 0;
	PD_InfoProbe_Active = 0;
	PD_InfoProbe_TxRetry = 0;
	PD_InfoProbe_NoResponseCnt = 0;
	PD_InfoProbe_Done = 0;
	PD_PostExit_Request_DelayMs = 0;
	PD_EPR_Probe_Done = 0;
	PD_PPS_Probe_Done = 0;
	PD_InfoProbe_SoftRst_Recovery_5V_Cnt = 0;
	s_epr_sinkcap_sent = 0;
	PD_Probed_SrcCap_Fingerprint = 0;  /* 物理切断: 再接続時に再プローブを許可 */
	PD_EPR_Attempted_FP = 0;           /* 物理切断: 同じ PDO セットの EPR 再測定を許可 */
	VDM_Reset_Disc_State();
	PD_Source_VDM_Probe_Reset();
	PD_User_DevOut();
	PD_Result_OnDetach();
}

/*********************************************************************
 * @fn      pProt_IDLE
 *
 * @brief   Message reception without state machine.
 *
 * @return  none
 */
void pProt_IDLE(void)
{
	st_VDM_Header *vdm = (st_VDM_Header *)&PD_RX_BUF[1];

	VDM_Sniff_SOPP_Cable();

	if ( PD_PostExit_Request_DelayMs != 0u &&
	     PD_PHY.LastRxSop == PD_PHY_RX_SOP &&
	     !rxHeader->Extended && rxHeader->NDO > 0u &&
	     rxHeader->MsgType == PD_Data_VendorDefined ) {
		if ( vdm->Type && vdm->CommandType == 0u &&
		     vdm->Command == PD_VDM_DiscoverSVIDs &&
		     vdm->SVID == 0xFF00u ) {
			printf("RX Discover SVIDs during Request guard; "
			       "hardware GoodCRC only\r\n");
			PD_PostExit_Request_DelayMs = 15u;
			PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
			return;
		}
	}

	if ( rxHeader->Extended ) {
		if ( rxExtHeader->Chunked ) DEBUG_Print("rxDataSize.%d\r\nrxChunkNumber.%d\r\n",rxExtHeader->DataSize,rxExtHeader->ChunkNumber);
		if ( PD_Is_Extended_Chunk0_With_More_Data() &&
		     rxHeader->MsgType == PD_Ext_EPRSrcCapabilities ) {
			printf("RX EPR_Source_Capabilities chunk 0 while idle; request chunk 1\r\n");
			PD_EPR_SrcCap_RX();
			return;
		}
		if ( rxExtHeader->ChunkNumber == 0 && PD_Is_Get_Request_Extended(rxHeader->MsgType) ) {
			printf("RX unexpected Extended GET_* 0x%02X; TX NotSupported\r\n", rxHeader->MsgType);
			PD_TX_NotSupported();
			return;
		}
		if ( rxExtHeader->Chunked && ( rxExtHeader->DataSize > ( rxExtHeader->ChunkNumber*26 + rxHeader->NDO*4 - 2 ) ) ) PD_TX_ChunkRequest();
		else if ( rxExtHeader->ChunkNumber == 0 ) {
			if ( rxHeader->MsgType == PD_Ext_GetBatteryCap )		PD_TX_NotSupported();
			if ( rxHeader->MsgType == PD_Ext_GetBatteryStatus )		PD_TX_NotSupported();
			if ( rxHeader->MsgType == PD_Ext_SecurityRequest )		PD_TX_NotSupported();
			if ( rxHeader->MsgType == PD_Ext_GetManufacturerInfo )	PD_TX_ManufacturerInfo();
		}
		else PD_RX_Reserved();
	}
	//Data Message
	else if ( rxHeader->NDO ) {
		if ( Check_DataMsg_Type_Reserved ) PD_RX_Reserved();		//Protocol limit of PD2.0 or 3.0
		else ( Idle_DataMsg_Handle[ enum_Idle_DataMsg[rxHeader->MsgType] ][ (PD_PHY.Header.PortPwrRole)?(0):(1) ] )();
	}
	//Control Message
	else {
		if ( PD_Is_Get_Request_Control(rxHeader->MsgType) ) {
			printf("RX unexpected GET_* %s; TX NotSupported\r\n", PD_Ctrl_Msg_Name(rxHeader->MsgType));
			PD_TX_NotSupported();
			return;
		}
		if ( Check_CtrlMsg_Type_Reserved ) PD_RX_Reserved();		//Protocol limit of PD2.0 or 3.0
		else ( Idle_CtrlMsg_Handle[ enum_Idle_CtrlMsg[rxHeader->MsgType] ][ (PD_PHY.Header.PortPwrRole)?(0):(1) ] )();
	}
}

/*********************************************************************
 * @fn      pProt_RX_DRSwap
 *
 * @brief   Receive DRSwap,Switch data roles.
 *
 * @return  none
 */
void pProt_RX_DRSwap(void)
{
  if( !(VDM_State.Enter_Mode_already) )
  {
	  if(PD_PHY.Header.PortPwrRole)
	  {
		PD_PHY_Header_Init(1,0,(PD_PHY.Header.PortDataRole)?(PD_Ctrl_Accept):(PD_Ctrl_Reject));
		PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
		PD_PHY.Header.PortDataRole = 1-PD_PHY.Header.PortDataRole;
	  }
	  else
	  {
		  PD_TX_Reject();
	  }
  }
  else
  {
	  PD_TX_HRST();
  }
}

/*********************************************************************
 * @fn      pProt_TX_SinkCap
 *
 * @brief   Send SinkCap.
 *
 * @return  none
 */
void pProt_TX_SinkCap(void)
{
	PD_PHY_Header_Init(1,SinkCapCnt,PD_Data_SinkCap);		//Send SinkCap 1ms later
	memcpy(&PD_TX_BUF[1],SinkCap,SinkCapCnt*4);
	//	Transmission successful pProt_TX_PS_RDY		Received successfully; analyze whether it is a request		Send failed SoftRST		Receive timeout SoftRST
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}

/*********************************************************************
 * @fn      pProt_TX_SrcCap
 *
 * @brief   Send SrcCap.
 *
 * @return  none
 */
void pProt_TX_SrcCap(void)
{
	PD_PHY_Header_Init(1,SrcCapCnt,PD_Data_SrcCap);		//Send SrcCap after 1ms
	memcpy(&PD_TX_BUF[1],SrcCap,SrcCapCnt*4);
	if ( PD_PHY.SrcCapCnt ) {	//SrcCap at startup: 120ms interval, no reset on failure to send
		PD_PHY.SrcCapCnt --;
		PD_PHY.WaitMsgRx = 0;
		PD_PHY.MsgTxCnt = 120;
		//PD_PHY.TxMsgID ++;
		PD_Prot_pSet( pProt_Wait_Request , (PD_PHY.SrcCapCnt)?(pProt_RX_Request):(pProt_IDLE) , (PD_PHY.SrcCapCnt)?(pProt_TX_SrcCap):(NULL) , NULL );
	}
	else if ( PD_PHY.Header.PortPwrRole ) pProt_Wait_Request();			//SrcCap in Source protocol: Send immediately, send failure SoftRST
	else PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , pProt_TX_SoftRst );
	//	Send success don't care   Receive success jump to pProt_RX_Request     Send failure SoftRST      Receive timeout SoftRST
}

/*********************************************************************
 * @fn      pProt_Wait_Request
 *
 * @brief   Wait Request.
 *
 * @return  none
 */
void pProt_Wait_Request(void)
{
	//Receive Request within 30ms
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 28;
	PD_Prot_pSet( NULL , pProt_RX_Request , pProt_TX_SoftRst , PD_TX_HRST );
	//	Send success don't care   Receive success jump to pProt_RX_Request   Send failure SoftRST  Receive timeout SoftRST
}

/*********************************************************************
 * @fn      pProt_RX_Request
 *
 * @brief   Receive Request.
 *
 * @return  none
 */
void pProt_RX_Request(void)
{
	if ( rxHeader->MsgType == PD_Data_Request ) {
		st_Request_Fixed tRequest;
		PD_PHY.SrcCapCnt = 0;
		PD_PHY.WaitMsgRx = 0;
		tRequest.Data = (PD_RX_BUF[2]<<16) + PD_RX_BUF[1];
		if ( !tRequest.ObjectPos || ( tRequest.ObjectPos > (sizeof(SrcCap)/4) ) ) PD_TX_Reject();		//First judge ObjectPos separately to prevent overflow when initializing the structure.
		else {
			st_SrcCap_Fixed *tSrcCap = (st_SrcCap_Fixed *)&SrcCap[tRequest.ObjectPos-1];		//Create the parser structure
			if ( tRequest.Current > tSrcCap->MaxCurrent ) {
				DEBUG_Print("Req Reject:%d,%d,%d,%d\r\n",tRequest.ObjectPos,tRequest.Current,tRequest.MaxCurrent,tSrcCap->MaxCurrent);
				PD_TX_Reject();
			}
			else {
				PD_PHY.savedRequest.Data = tRequest.Data;
				pProt_TX_Accept();
			}
		}
	}
	else {
		PD_PHY.WaitMsgRx = 0;
		pProt_TX_SoftRst();
	}
}

/*********************************************************************
 * @fn      pProt_TX_Accept
 *
 * @brief   Send Accept.
 *
 * @return  none
 */
void pProt_TX_Accept(void)
{
	DEBUG_Print("TX Accept\r\n");
	PD_PHY_Header_Init(1,0,PD_Ctrl_Accept);		//Send Accept after 1ms

	//	Send success pProt_TX_PS_RDY; Receive success analyze whether it is a request; Send failure SoftRST; Receive timeout SoftR
	PD_Prot_pSet( pProt_Set_Volt_Change , pProt_RX_Request , pProt_TX_SoftRst , pProt_TX_SoftRst );

	//	Send success pProt_TX_PS_RDY; Receive success analyze whether it is a request; Send failure SoftRST; Receive timeout SoftRST
	//PD_Prot_pSet( pProt_TX_PS_RDY , pProt_RX_Request , pProt_TX_SoftRst , pProt_TX_SoftRst );
}

/*********************************************************************
 * @fn      pProt_Set_Volt_Change
 *
 * @brief   Set the conditions for voltage change.
 *
 * @return  none
 */
void pProt_Set_Volt_Change(void)
{
	PD_PHY.VoltChanging = 1;
	PD_PHY.tSrcTransition = 35;
	PD_PHY.tPSTransition = 470;
}

/*********************************************************************
 * @fn      pProt_Volt_Change
 *
 * @brief   Voltage change.
 *
 * @return  none
 */
void pProt_Volt_Change(void)
{
	PD_User_Src_VoltChange();
}

/*********************************************************************
 * @fn      pProt_TX_PS_RDY
 *
 * @brief   Send PS_RDY.
 *
 * @return  none
 */
void pProt_TX_PS_RDY(void)
{
	PD_PHY.VoltChanging = 0;
	VDM_State.Explicit_Contract_Established = 1;
	PD_PHY_Header_Init(35,0,PD_Ctrl_PS_Ready);		//Send Accept after 1ms
	//	Send success pProt_TX_PS_RDY; Receive success analyze whether it is a request; Send failure SoftRST; Receive timeout SoftRST
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
}

/*********************************************************************
 * @fn      pProt_Wait_SrcCap
 *
 * @brief   Wait SrcCap.
 *
 * @return  none
 */
void pProt_Wait_SrcCap(void)
{
//Received Accept within 465ms
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 465;
//	Send success don't care; Receive success analyze whether it is Accept; Send failure NULL; Receive timeout SoftRST.
	PD_Prot_pSet( NULL , pProt_RX_SrcCap , NULL , PD_TX_HRST );
}

/*********************************************************************
 * @fn      pProt_RX_SrcCap
 *
 * @brief   Receive SrcCap.
 *
 * @return  none
 */
void pProt_RX_SrcCap(void)
{
	u8 previous_epr_cap = PD_Stored_SrcCap_EprCap;

	VDM_Sniff_SOPP_Cable();

	if ( rxHeader->MsgType == PD_Data_SrcCap ) {
		if ( rxHeader->NDO == 0 || rxHeader->NDO > 7 ) {
			printf("RX invalid Source_Capabilities NDO:%d; ignore without Request\r\n", rxHeader->NDO);
			PD_PHY.WaitMsgRx = 0;
			PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
			return;
		}
#if PD_USE_ANALYZER
		PD_Analyzer_On_Valid_SrcCap( rxHeader->NDO );
#endif
		PD_Cable_Sniff_On_SrcCap();
		PD_PHY.rxSrcCapCnt = rxHeader->NDO * 4;
		memset(PD_PHY.rxSrcCap, 0, sizeof(PD_PHY.rxSrcCap));
		memcpy(PD_PHY.rxSrcCap, &PD_RX_BUF[1], PD_PHY.rxSrcCapCnt);
		PD_Current_SrcCap_Fingerprint = PD_SourceCap_Fingerprint();

		/*
		 * MsgID=0 は新規 PD セッション開始（物理再接続 / HardReset 後）を示す。
		 * ただし EPR_Mode Enter に対して多くのソースは SoftReset で応答し、
		 * 同一 PDO で MsgID=0 を再送してくる（物理的な切断はない）。
		 * その場合フィンガープリントが一致するので、EPR 済みフラグを保持して
		 * 無限 EPR ループを防ぐ。
		 * フィンガープリントが変わった場合（別デバイス接続 or AOHI 二相目）は
		 * 両方クリアして新規プローブを許可する。
		 */
		if ( rxHeader->MsgID == 0 ) {
			if ( PD_Current_SrcCap_Fingerprint != PD_Probed_SrcCap_Fingerprint ) {
				/* 別デバイス or フィンガープリント未登録 → 完全リセット */
				PD_Probed_SrcCap_Fingerprint = 0;
				PD_InfoProbe_Done            = 0;
				PD_EPR_Probe_Done            = 0;
			}
			/* 同一フィンガープリント (EPR 起因リセット) → 両フラグを保持してループ抑制 */
		}

#if !PD_ANALYZER_NEGOTIATE_CONTRACT
		/* ── 非コントラクトモード: 即時表示して終了 ── */
		if ( PD_Last_Printed_SrcCap_Valid && (rxHeader->NDO == PD_Last_Printed_SrcCap_NDO) && (PD_Current_SrcCap_Fingerprint == PD_Last_Printed_SrcCap_Fingerprint) ) {
			PD_Suppressed_Same_SrcCap_Count++;
			if ( PD_Suppressed_Same_SrcCap_Count == 1 ) {
				printf("RX same Source_Capabilities muted after first repeat MsgID:%d NDO:%d FP:0x%08lX\r\n", rxHeader->MsgID, rxHeader->NDO, (unsigned long)PD_Current_SrcCap_Fingerprint);
			}
			PD_PHY.WaitMsgRx = 0;
			PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
			return;
		}
		PD_Last_Printed_SrcCap_Valid = 1;
		PD_Last_Printed_SrcCap_NDO = rxHeader->NDO;
		PD_Last_Printed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
		PD_Suppressed_Same_SrcCap_Count = 0;
		PD_Print_Source_Capabilities();
		printf("Analyzer no-contract mode: Source_Capabilities captured; Request/probes skipped for stable PDO dump.\r\n");
		PD_PHY.WaitMsgRx = 0;
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );

#else
		/* ── コントラクトモード ── */

		/* 既にコントラクト確立済みでも、ソース再広告には Fixed PDO1 を再 Request。
		 * aohi-240w は PS_RDY 直後に SrcCap を再送する — 無視すると SoftReset/HRST 嵐になる。 */
		if ( VDM_State.Explicit_Contract_Established ) {
			PD_Stored_SrcCap_NDO     = rxHeader->NDO;
			PD_Stored_SrcCap_SpecRev = rxHeader->SpecRevision;
			PD_Stored_SrcCap_MsgID   = rxHeader->MsgID;
			PD_Stored_SrcCap_EprCap  = (u8)((PD_PHY.rxSrcCap[0] >> 23) & 1);
			PD_PHY.Header.SpecRevision = rxHeader->SpecRevision;

			if ( !previous_epr_cap && PD_Stored_SrcCap_EprCap ) {
				printf("RX Source_Capabilities upgraded to EPR-capable set; continue PPS/EPR path.\r\n");
				PD_Source_VDM_Probe_Cancel();
				PD_PPS_Probe_Done = 0;
				PD_EPR_Probe_Done = 0;
				/* Interim Cap had EPRCap=0 so SinkCap was deferred — allow it now. */
				s_epr_sinkcap_sent = 0;
				PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
				PD_EPR_Enter_Probe_If_Capable();
				return;
			}

			printf("RX Source_Capabilities after contract — Request Fixed PDO1\r\n");
			PD_User_Snk_Rx_SrcCap();
			if ( PD_InfoProbe_Done || PD_EPR_Probe_Done ) {
				/*
				 * LA280PM240 starts a source-initiated Discover SVIDs AMS about
				 * 0.4ms after Source_Capabilities.  Hold Request briefly so
				 * the source AMS can receive its empty ACK, but stay below the
				 * source's approximately 30 ms transition deadline.
			 */
			VDM_State.Explicit_Contract_Established = 0;
			PD_PostExit_Request_DelayMs = 15u;
				PD_PHY.WaitMsgRx = 0u;
				PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
				printf("Source_Capabilities after completed probes: hold Request 15ms for source AMS\r\n");
			} else {
				pProt_TX_Request();
			}
			return;
		}

		/* 直前の Request 失敗と同じ SrcCap なら抑制 */
		if ( PD_Request_Stop_After_Fail ) {
			if ( (rxHeader->NDO != PD_Failed_SrcCap_NDO) || (PD_Current_SrcCap_Fingerprint != PD_Failed_SrcCap_Fingerprint) ) {
				printf("RX Source_Capabilities changed after Request failure; clearing stop flag and retrying.\r\n");
				PD_Request_Stop_After_Fail = 0;
				PD_Request_Fail_Count = 0;
				PD_Suppressed_SrcCap_Count = 0;
			} else {
				PD_Suppressed_SrcCap_Count++;
				if ( (PD_Suppressed_SrcCap_Count == 1) || ((PD_Suppressed_SrcCap_Count & 0x0F) == 0) ) {
					printf("RX Source_Capabilities suppressed after Request failure count=%u MsgID:%d NDO:%d FP:0x%08lX\r\n",
						PD_Suppressed_SrcCap_Count, rxHeader->MsgID, rxHeader->NDO, (unsigned long)PD_Current_SrcCap_Fingerprint);
				}
				PD_PHY.WaitMsgRx = 0;
				PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
				return;
			}
		}

		/*
		 * ★ タイミング修正: Source_Cap 受信から Request 送信まで printf を一切行わない。
		 *   printf は UART ブロッキング送信であり、460800 baud でも数 ms/行 かかる。
		 *   複数行を印字すると tSenderResponse (30ms) を超過して GoodCRC が返らなくなる。
		 *   情報は一時変数に保存し、PS_RDY 受信後または失敗時に遅延表示する。
		 */
		PD_Stored_SrcCap_NDO     = rxHeader->NDO;
		PD_Stored_SrcCap_SpecRev = rxHeader->SpecRevision;
		PD_Stored_SrcCap_MsgID   = rxHeader->MsgID;
		PD_Stored_SrcCap_EprCap  = (u8)((PD_PHY.rxSrcCap[0] >> 23) & 1);

		PD_PHY.Header.SpecRevision = rxHeader->SpecRevision;

		PD_User_Snk_Rx_SrcCap();

		/* Request を即座に送信 (遅延なし・printf なし) */
		pProt_TX_Request();
#endif
	}
	else if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
	          rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
	}
	else pProt_IDLE();
}

static void pProt_Request_Failed_Common(const char *reason)
{
	PD_Request_Fail_Count++;
	PD_Request_Stop_After_Fail = 1;
	PD_Suppressed_SrcCap_Count = 0;
	PD_Failed_SrcCap_NDO         = PD_Request_SrcCap_NDO;
	PD_Failed_SrcCap_Fingerprint = PD_Request_SrcCap_Fingerprint;

	/* 失敗時に遅延表示: Source_Cap → Request の内容を診断情報として出力 */
	PD_Print_Source_Capabilities_Deferred();
	PD_Print_Request(PD_Stored_Request_RDO, PD_Stored_Selected_PDO);
	if ( PD_InfoProbe_SoftRst_Recovery_5V_Cnt ) {
		printf("Request note: probe SoftReset recovery forced PDO1/5V to stabilize source\r\n");
		PD_InfoProbe_SoftRst_Recovery_5V_Cnt = 0;
	}

	printf("Request stage failed #%d: %s. Stop retry until SourceCap changes or SoftReset. failFP:0x%08lX NDO:%d\r\n",
		PD_Request_Fail_Count, reason, (unsigned long)PD_Failed_SrcCap_Fingerprint, PD_Failed_SrcCap_NDO);
	PD_PHY.WaitMsgRx = 0;
	PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );
}

static void pProt_Request_TxFailed(void)
{
	pProt_Request_Failed_Common("Request TX failed: no GoodCRC for Request");
}

static void pProt_Request_RxTimeout(void)
{
	pProt_Request_Failed_Common("Request RX timeout: no Accept/Reject/Wait before timeout");
}

static void pProt_PS_RDY_Failed(void)
{
	PD_Print_Source_Capabilities_Deferred();
	PD_Print_Request(PD_Stored_Request_RDO, PD_Stored_Selected_PDO);
	if ( PD_InfoProbe_SoftRst_Recovery_5V_Cnt ) {
		printf("Request note: probe SoftReset recovery forced PDO1/5V to stabilize source\r\n");
		PD_InfoProbe_SoftRst_Recovery_5V_Cnt = 0;
	}
	printf("PS_RDY stage failed: timeout after Accept. HardReset.\r\n");
	PD_PHY.WaitMsgRx = 0;
	PD_TX_HRST();
}

#define PD_REQUEST_ASYNC_RX_MAX 8u

static u8 pProt_Request_Rx_Is_PostTx(void)
{
	/* Sequence zero is retained as a compatibility fallback for direct RX. */
	return (u8)(PD_PHY.LastRxSequence == 0u ||
	            PD_PHY.LastRxSequence > PD_PHY.TxStartRxSequence);
}

static void pProt_Request_Rearm_After_Async(u8 waiting_ps_rdy)
{
	const char *phase = waiting_ps_rdy ? "PS_RDY" : "Accept";
	const char *arrival;
	st_VDM_Header *vdm = (st_VDM_Header *)&PD_RX_BUF[1];
	u8 is_vdm;
	u8 is_discover_svid;
	u32 age_ms;

	PD_Request_Async_Rx_Count++;
	is_vdm = (u8)(PD_PHY.LastRxSop == PD_PHY_RX_SOP &&
	              !rxHeader->Extended && rxHeader->NDO > 0u &&
	              rxHeader->MsgType == PD_Data_VendorDefined);
	is_discover_svid = (u8)(is_vdm && vdm->Type &&
	                        vdm->CommandType == 0u &&
	                        vdm->Command == PD_VDM_DiscoverSVIDs &&
	                        vdm->SVID == 0xFF00u);
	arrival = (PD_PHY.LastRxSequence != 0 &&
	           PD_PHY.LastRxSequence <= PD_PHY.TxStartRxSequence)
	        ? "pre-TX queued" : "post-TX async";
	age_ms = PD_PHY.TimeMs - PD_PHY.LastRxTimestampMs;

	printf("RX unrelated while waiting %s: MsgType:0x%02X NDO:%d Extended:%d "
	       "seq:%lu txStartSeq:%lu rxAt:%lums txAt:%lums age:%lums (%s); keep waiting\r\n",
		phase, rxHeader->MsgType, rxHeader->NDO, rxHeader->Extended,
		(unsigned long)PD_PHY.LastRxSequence,
		(unsigned long)PD_PHY.TxStartRxSequence,
		(unsigned long)PD_PHY.LastRxTimestampMs,
		(unsigned long)PD_PHY.TxStartTimestampMs,
		(unsigned long)age_ms, arrival);

	/*
	 * LA280PM240 injects Discover SVIDs around Request/Accept and Hard Resets
	 * after ACK, NAK, or BUSY from this analyzer.  The known-good trace has
	 * only the PHY-generated GoodCRC, followed by continued Request handling.
	 */
	if ( is_discover_svid ) {
		printf("Quarantine asynchronous Discover SVIDs during Request %s wait; "
		       "hardware GoodCRC only and keep waiting\r\n", phase);
	} else if ( is_vdm ) {
		printf("Quarantine asynchronous VDM during Request %s wait; "
		       "hardware GoodCRC only\r\n", phase);
	}

	if ( PD_Request_Async_Rx_Count > PD_REQUEST_ASYNC_RX_MAX ) {
		if ( waiting_ps_rdy ) {
			pProt_PS_RDY_Failed();
		} else {
			pProt_Request_Failed_Common(
				"too many asynchronous messages while waiting for Request response");
		}
		return;
	}

	PD_PHY.WaitMsgRx = 1;
	if ( waiting_ps_rdy ) {
		/*
		 * Dell's ordinary EPR transition needs about 737 ms, and its
		 * source-initiated VDM can further delay the SPR PS_RDY.
		 */
		PD_PHY.MsgRxCnt = is_vdm ? 1500u : 500u;
		PD_Prot_pSet( NULL , pProt_RX_PS_RDY , NULL , pProt_PS_RDY_Failed );
	} else {
		PD_PHY.MsgRxCnt = 60;
		PD_Prot_pSet( NULL , pProt_RX_Accept , NULL , pProt_Request_RxTimeout );
	}
}

/*********************************************************************
 * @fn      pProt_TX_Request
 *
 * @brief   Send Request.
 *
 * @return  none
 */
void pProt_TX_Request(void)
{
	st_Request_Fixed *pRequest  = (st_Request_Fixed *)&PD_TX_BUF[1];
	st_SrcCap_Fixed  *pSrcCap;
	u8  ndo         = PD_Stored_SrcCap_NDO;
	u16 req_current;

	/* PD_Stored_SrcCap_NDO は pProt_RX_SrcCap 内で設定済み */
	PD_Request_SrcCap_NDO         = ndo;
	PD_Request_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
	PD_Request_Async_Rx_Count     = 0;

	/*
	 * The proven analyzer sequence establishes its SPR contract on PDO1/5V.
	 * SinkCap[0] advertises 20V only to declare EPR capability; it is not the
	 * desired operating voltage.  Staying at 5V also avoids a power transition
	 * racing the PPS/SinkCap/EPR probe sequence.
	 */
	pRequest->Data      = 0;
	pRequest->ObjectPos = 1;
	if ( PD_InfoProbe_SoftRst_Recovery_5V_Cnt != 0 ) {
		PD_InfoProbe_SoftRst_Recovery_5V_Cnt--;
	}
	pSrcCap = (st_SrcCap_Fixed *)&PD_PHY.rxSrcCap[pRequest->ObjectPos - 1];
	pRequest->USBComm      = 0;
	pRequest->NoUSBSuspend = 1;
	/* SinkCap[0] に EPRCap=1 が立っているので、RDO の EPR Mode Capable bit を宣言する。
	 * ソースはこのビットを見て EPR_Mode Enter を許可するかどうか判断する。
	 * bit23 Unchunked Ext も立てる（シンプル版 RDO 0x11C4xxxx と一致）。 */
	pRequest->EPRMode            = 1;
	pRequest->UnchunkedExtended  = 1;

	req_current = pSrcCap->MaxCurrent;
#if PD_ANALYZER_INITIAL_REQUEST_MA
	{
		u16 analyzer_limit = PD_ANALYZER_INITIAL_REQUEST_MA / 10;
		if ( req_current > analyzer_limit ) req_current = analyzer_limit;
		if ( req_current == 0 )            req_current = 1;
	}
#endif
	pRequest->Current = pRequest->MaxCurrent = req_current;

	PD_PHY.savedRequest.Data = pRequest->Data;

	/* 遅延表示用に保存 */
	PD_Stored_Request_RDO       = pRequest->Data;
	PD_Stored_Request_ObjectPos = pRequest->ObjectPos;
	PD_Stored_Selected_PDO      = PD_PHY.rxSrcCap[pRequest->ObjectPos - 1];

	/*
	 * ★★ タイミング最重要: ここで即座に Request を送信。
	 *   PD_PHY_Header_Init の printf("tx") はあるが、FlushTxNow で即 BMC に出す。
	 *   Cap dump は PS_RDY 後（遅延表示）。
	 */
	PD_PHY_Header_Init(0, 1, PD_Data_Request);
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt  = 60;
	PD_Prot_pSet( NULL , pProt_RX_Accept , pProt_Request_TxFailed , pProt_Request_RxTimeout );
	PD_PHY_FlushTxNow();

	/*
	 * TX Request の詳細ログは Accept/PS_RDY 受信後または失敗時に
	 * Source_Cap と一緒に一括表示するため、ここでは出力しない。
	 */
}

/*********************************************************************
 * @fn      pProt_RX_Accept
 *
 * @brief   Receive Accept.
 *
 * @return  none
 */
void pProt_RX_Accept(void)
{

//Check if it is Accept
	DEBUG_Print("Check Accept\r\n");
	if ( pProt_Request_Rx_Is_PostTx() &&
	     !rxHeader->Extended && rxHeader->NDO == 0 &&
	     rxHeader->MsgType == PD_Ctrl_Accept ) {
		/* Accept 受信を記録 (詳細は PS_RDY 受信後に一括表示) */
		DEBUG_Print("RX Accept for Request\r\n");
//Received PS_RDY within 500ms
		PD_PHY.WaitMsgRx = 1;
		PD_PHY.MsgRxCnt = 500;
//	Send success don't care; Receive success analyze whether it is PS_RDY; No send; Receive timeout HRST
		PD_Prot_pSet( NULL , pProt_RX_PS_RDY , NULL , pProt_PS_RDY_Failed );
	}
	else if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
	          rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
	}
	else if ( !rxHeader->Extended && rxHeader->NDO > 0 &&
	          rxHeader->MsgType == PD_Data_SrcCap ) {
		/* Source re-advertised instead of Accept — Request Fixed PDO1 again. */
		printf("RX Source_Capabilities while waiting Accept — re-Request Fixed PDO1\r\n");
		PD_PHY.rxSrcCapCnt = rxHeader->NDO * 4;
		memset(PD_PHY.rxSrcCap, 0, sizeof(PD_PHY.rxSrcCap));
		memcpy(PD_PHY.rxSrcCap, &PD_RX_BUF[1], PD_PHY.rxSrcCapCnt);
		PD_Stored_SrcCap_NDO     = rxHeader->NDO;
		PD_Stored_SrcCap_SpecRev = rxHeader->SpecRevision;
		PD_Stored_SrcCap_MsgID   = rxHeader->MsgID;
		PD_Stored_SrcCap_EprCap  = (u8)((PD_PHY.rxSrcCap[0] >> 23) & 1);
		PD_PHY.Header.SpecRevision = rxHeader->SpecRevision;
		PD_Current_SrcCap_Fingerprint = PD_SourceCap_Fingerprint();
		pProt_TX_Request();
	}
	else if ( pProt_Request_Rx_Is_PostTx() &&
	          !rxHeader->Extended && rxHeader->NDO == 0 &&
	          rxHeader->MsgType == PD_Ctrl_Reject ) {
		pProt_Request_Failed_Common("Source rejected Request");
	}
	else if ( pProt_Request_Rx_Is_PostTx() &&
	          !rxHeader->Extended && rxHeader->NDO == 0 &&
	          rxHeader->MsgType == PD_Ctrl_Wait ) {
		pProt_Request_Failed_Common("Source returned Wait for Request");
	}
	else if ( pProt_Request_Rx_Is_PostTx() &&
	          !rxHeader->Extended && rxHeader->NDO == 0 &&
	          rxHeader->MsgType == PD_Ctrl_NotSupported ) {
		pProt_Request_Failed_Common("Source returned Not_Supported for Request");
	}
	else {
		pProt_Request_Rearm_After_Async(0);
	}
}

/*********************************************************************
 * @fn      pProt_RX_PS_RDY
 *
 * @brief   Receive PS_RDY.
 *
 * @return  none
 */
void pProt_RX_PS_RDY(void)
{
	DEBUG_Print("Check PS_RDY\r\n");
	if ( pProt_Request_Rx_Is_PostTx() &&
	     !rxHeader->Extended && rxHeader->NDO == 0 &&
	     rxHeader->MsgType == PD_Ctrl_PS_Ready ) {
		DEBUG_Print("ADC VBUS:%.2f\r\n",(float)GetADC_VBUS/4096*3.3/33*233);
		PD_PHY.WaitMsgRx = 0;
		VDM_State.Explicit_Contract_Established = 1;
		s_pd_result.spr_contract_status = PD_RESULT_STATUS_PASS;
		s_pd_result_dirty |= PD_RESULT_DIRTY_PROTOCOL;

		/* 契約確立後に遅延表示: Source_Cap → Request → 結果 の順に出力 */
		PD_Print_Source_Capabilities_Deferred();
		PD_Print_Request(PD_Stored_Request_RDO, PD_Stored_Selected_PDO);
		if ( PD_InfoProbe_SoftRst_Recovery_5V_Cnt ) {
			printf("Request note: probe SoftReset recovery forced PDO1/5V to stabilize source\r\n");
			PD_InfoProbe_SoftRst_Recovery_5V_Cnt = 0;
		}
		printf("RX Accept + PS_RDY: Explicit contract established. PDO Index:%d RDO:0x%08lX\r\n",
			PD_PHY.savedRequest.ObjectPos, (unsigned long)PD_PHY.savedRequest.Data);

		PD_User_Snk_Rx_PS_RDY();
		PD_Prot_pSet( NULL , pProt_IDLE , NULL , NULL );

		/*
		 * 同一ソース再接続の検出: フィンガープリントが一致する場合はプローブ済みのため
		 * 再プローブをスキップする（ソースが Discover Identity 等に反応して物理リセットを
		 * 繰り返す無限ループを防止）。別ソースや初回接続時は fingerprint が異なるため通常通りプローブ。
		 */
		if ( PD_InfoProbe_Done ||
		     ( PD_Probed_SrcCap_Fingerprint != 0
		    && PD_Current_SrcCap_Fingerprint == PD_Probed_SrcCap_Fingerprint ) ) {
			PD_InfoProbe_Done = 1;
			PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
			printf("Probe done for this PDO set; skip GET_* and try EPR path.\r\n");
			/* プローブ済みでも EPR が未試行なら試みる
			 * (プローブ途中で SoftReset が入り probe skipped になったケースに対応) */
			if ( PD_EPR_Probe_Done == 0 ) {
				PD_EPR_Enter_Probe_If_Capable();
			} else {
				/* EPR Exit re-contract is complete; source VDM probing is now safe. */
				PD_Source_VDM_Probe_Arm_Delayed(250);
			}
		} else {
			PD_InfoProbe_Start();
		}
	}
	else if ( !rxHeader->Extended && rxHeader->NDO == 0 &&
	          rxHeader->MsgType == PD_Ctrl_SoftReset ) {
		pProt_RX_SoftRst();
	}
	else if ( !rxHeader->Extended && rxHeader->NDO > 0 &&
	          rxHeader->MsgType == PD_Data_SrcCap ) {
		printf("RX Source_Capabilities while waiting PS_RDY; restart Request transaction\r\n");
		pProt_RX_SrcCap();
	}
	else if ( pProt_Request_Rx_Is_PostTx() &&
	          !rxHeader->Extended && rxHeader->NDO == 0 &&
	          (rxHeader->MsgType == PD_Ctrl_Reject ||
	           rxHeader->MsgType == PD_Ctrl_Wait ||
	           rxHeader->MsgType == PD_Ctrl_NotSupported) ) {
		pProt_Request_Failed_Common("Source terminated Request after Accept");
	}
	else {
		pProt_Request_Rearm_After_Async(1);
	}
}

/*********************************************************************
 * @fn      pProt_TX_SoftRst
 *
 * @brief   Send Software reset.
 *
 * @return  none
 */
void pProt_TX_SoftRst(void)
{
	DEBUG_Print("TX SoftRST\r\n");
	PD_PostExit_Request_DelayMs = 0;
	PD_PHY.TxMsgID = PD_PHY.RxMsgID = 0;
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;
	/* SoftReset 後に Request を再試行できるようフラグをリセット */
	PD_Request_Stop_After_Fail = 0;
	PD_Request_Fail_Count      = 0;
	PD_Suppressed_SrcCap_Count = 0;
	PD_PHY_Header_Init(1,0,PD_Ctrl_SoftReset);
//Accept within 50ms
	PD_PHY.WaitMsgRx = 1;
	PD_PHY.MsgRxCnt = 50;
	PD_Prot_pSet( NULL , pProt_SoftRst_RX_Accept , NULL , PD_TX_HRST );
}

/*********************************************************************
 * @fn      pProt_SoftRst_RX_Accept
 *
 * @brief   Receive Software reset Accept.
 *
 * @return  none
 */
void pProt_SoftRst_RX_Accept(void)
{
	DEBUG_Print("Check Accept\r\n");
	PD_PHY.WaitMsgRx = 0;
	if ( rxHeader->MsgType == PD_Ctrl_Accept ) pProt_Excute_SoftRst();
	else PD_TX_HRST();
}

/*********************************************************************
 * @fn      pProt_RX_SoftRst
 *
 * @brief   Receive Software reset.
 *
 * @return  none
 */
void pProt_RX_SoftRst(void)
{
	DEBUG_Print("Rx SoftRST\r\n");
	PD_PostExit_Request_DelayMs = 0;
	/*
	 * Cancel the interrupted AMS before preparing Accept.  In particular,
	 * prevent a queued GET_* or chunk request from being emitted after the
	 * source has reset the protocol layer.
	 */
	USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
	USBPD->STATUS = 0xFFu;
	PD_PHY.WaitTxGcrc = 0;
	PD_PHY.WaitRxGcrc = 0;
	PD_PHY.WaitMsgTx = 0;
	PD_PHY.WaitMsgRx = 0;
	PD_PHY.MsgTxCnt = 0;
	PD_PHY.MsgRxCnt = 0;
	PD_PHY.pTxFinish = NULL;
	PD_PHY.pRxFinish = NULL;
	PD_PHY.pTxTimeout = NULL;
	PD_PHY.pRxTimeout = NULL;
	PD_PHY_Abort();

	if ( PD_InfoProbe_Skip_Mask != 0 || PD_InfoProbe_SoftRst_Cnt != 0 ) {
		PD_InfoProbe_SoftRst_Recovery_5V_Cnt = 3;
	}
	if ( PD_InfoProbe_Active || PD_InfoProbe_Step != 0 ) {
		PD_InfoProbe_Done = 1;
		PD_Probed_SrcCap_Fingerprint = PD_Current_SrcCap_Fingerprint;
	}
	PD_InfoProbe_Active = 0;
	PD_InfoProbe_Step = 0;
	PD_InfoProbe_TxRetry = 0;
	/* PD_InfoProbe_Done is deliberately preserved across protocol Soft Reset. */
	VDM_State.Explicit_Contract_Established = 0;
	VDM_State.Enter_Mode_already = 0;
	PD_PHY.TxMsgID = 0;
	PD_PHY.RxMsgID = 0;
	PD_PHY.TxSop = PD_PHY_TX_SOP;
	PD_PHY_Set_RxSop(PD_PHY_RX_SOP);
	PD_PHY_Set_ListenOnlySopp(0);
	/* SoftReset 受信後に Request を再試行できるようフラグをリセット */
	PD_Request_Stop_After_Fail = 0;
	PD_Request_Fail_Count      = 0;
	PD_Suppressed_SrcCap_Count = 0;
	PD_PHY_Header_Init(1,0,PD_Ctrl_Accept);
	PD_Prot_pSet( pProt_Excute_SoftRst , pProt_IDLE , NULL , NULL );
}

/*********************************************************************
 * @fn      pProt_Excute_SoftRst
 *
 * @brief   Excute Software reset.
 *
 * @return  none
 */
void pProt_Excute_SoftRst(void)
{
	DEBUG_Print("Excute SoftRst\r\n");
	if ( PD_DEVICE.DevStat ) {
		PD_PHY.SrcCapCnt = 50;
		pProt_TX_SrcCap();
		PD_PHY.MsgTxCnt = 450;
	}
	else pProt_Wait_SrcCap();
}

/*********************************************************************
 * @fn      pProt_RX_ChunkedMsg
 *
 * @brief   Receive ChunkedMsg.
 *
 * @return  none
 */
void pProt_RX_ChunkedMsg(void)
{
	PD_PHY_Header_Init(45,0,PD_Ctrl_NotSupported);
	PD_Prot_pSet( NULL , pProt_IDLE , pProt_TX_SoftRst , NULL );
}
