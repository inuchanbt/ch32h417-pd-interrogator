/********************************** (C) COPYRIGHT *******************************
 * pd_phy_h417.h — H417 PHY adapter for PD_Prot (replaces libCH32_USBPD.a + CH211).
 *
 * Migration goal: direct CC/BMC on PB3/PB4 — no I2C latency for SOP'/e-marker work.
 ********************************************************************************/

#ifndef PD_PHY_H417_H_
#define PD_PHY_H417_H_

#include "ch32h417_usbpd.h"
#include "pd_msgtypes.h"
#include "PD_Process.h"

#ifndef PD_USE_ANALYZER
#define PD_USE_ANALYZER  1
#endif

/*
 * nanoCH32H417 default: bridge R5/R6 so external 5.1k Rd is on CC1/CC2.
 * Set to 1 only when R5/R6 are open and the sink should present internal
 * USBPD CC_PD as a firmware-controlled Rd.
 */
#ifndef PD_SNK_USE_INTERNAL_RD
#define PD_SNK_USE_INTERNAL_RD  0
#endif

#define PD_PHY_RX_SOP    1u
#define PD_PHY_RX_SOPP   2u
#define PD_PHY_RX_SOPPP  3u
#define PD_PHY_TX_SOP    0x00u
#define PD_PHY_TX_SOPP   0x50u

typedef struct {
    union {
        uint16_t Data;
        struct {
            uint16_t MsgType:5;
            uint16_t PortDataRole:1;
            uint16_t SpecRevision:2;
            uint16_t PortPwrRole:1;
            uint16_t MsgID:3;
            uint16_t NDO:3;
            uint16_t Extended:1;
        };
    };
} st_Prot_Header;

typedef struct {
    union {
        uint16_t Data;
        struct {
            uint16_t DataSize:9;
            uint16_t Reserved:1;
            uint16_t RequestChunk:1;
            uint16_t ChunkNumber:4;
            uint16_t Chunked:1;
        };
    };
} st_Extended_Header;

typedef struct {
    union {
        uint32_t Data;
        struct {
            uint32_t MaxCurrent:10;
            uint32_t Current:10;
            uint32_t Reserved:2;
            uint32_t EPRMode:1;
            uint32_t UnchunkedExtended:1;
            uint32_t NoUSBSuspend:1;
            uint32_t USBComm:1;
            uint32_t CapbilityMismatch:1;
            uint32_t GiveBack:1;
            uint32_t ObjectPos:4;
        };
    };
} st_Request_Fixed;

typedef struct {
    void (*pDevChk)(void);
    void (*pUserUnattached)(void);
    void (*pUserAttached)(void);
    uint8_t  DevRole;
    uint8_t  DevStat;
    uint8_t  ConnectStat;
    uint8_t  VconnStat;
    uint16_t Cnt;
    uint16_t TryCnt;
    uint16_t Timeout;
} st_PD_DEVICE;

typedef struct {
    uint8_t  RxSop;
    uint8_t  LastRxSop;
    uint8_t  TxSop;
    uint8_t  TxMsgID;
    uint8_t  RxMsgID;
    uint8_t  RetryCnt;
    uint8_t  PDExist;
    uint16_t IdleCnt;
    uint8_t  SrcCapCnt;
    st_Prot_Header Header;
    uint8_t  WaitTxGcrc;
    uint8_t  WaitRxGcrc;
    uint8_t  WaitMsgTx;
    uint16_t MsgTxCnt;
    uint8_t  WaitMsgRx;
    uint16_t MsgRxCnt;
    uint8_t  VoltChanging;
    uint8_t  tSrcTransition;
    uint16_t tPSTransition;
    void (*pRxFinish)(void);
    void (*pRxTimeout)(void);
    void (*pTxFinish)(void);
    void (*pTxTimeout)(void);
    void (*pRxHRST)(void);
    uint32_t rxSrcCap[7];
    uint8_t  rxSrcCapCnt;
    st_Request_Fixed savedRequest;
    uint8_t  ListenOnlySopp;
} st_PD_PHY;

extern st_PD_PHY     PD_PHY;
extern st_PD_DEVICE  PD_DEVICE;

extern __attribute__((aligned(4))) uint16_t PD_RX_BUF[28];
extern __attribute__((aligned(4))) uint16_t PD_TX_BUF[28];

extern st_Prot_Header    *rxHeader;
extern st_Prot_Header    *txHeader;
extern st_Extended_Header *rxExtHeader;

extern uint32_t SrcCap[7];
extern vu8     SrcCapCnt;
extern uint32_t SinkCap[7];
extern vu8     SinkCapCnt;

extern uint16_t vSafe0V;
extern uint16_t vSinkDisconnect;
extern uint16_t vVBUSon;
extern uint16_t GetADC_VBUS;

#define Check_CtrlMsg_Type_Reserved \
    (rxHeader->MsgType > ((PD_PHY.Header.SpecRevision == PD_Rev3) ? 0x18u : PD_Ctrl_SoftReset))
#define Check_DataMsg_Type_Reserved \
    (((rxHeader->MsgType > ((PD_PHY.Header.SpecRevision == PD_Rev3) ? PD_Data_VendorDefined : PD_Data_SinkCap)) && \
      (rxHeader->MsgType != PD_Data_VendorDefined)))

void PD_PHY_H417_Init(void);
void PD_Analyzer_Init(void);
void PD_Analyzer_Det_Proc(void);
void Set_DevChk(uint8_t role);
void PD_PHY_Reset_Value(uint8_t role);
void PD_Prot_pSet(void (*pTxFinish)(void), void (*pRxFinish)(void),
                 void (*pTxTimeout)(void), void (*pRxTimeout)(void));
void PD_PHY_Set_RxSop(uint8_t sop);
void PD_PHY_Set_ListenOnlySopp(uint8_t on);
void PD_PHY_Header_Init(uint8_t cnt, uint8_t ndo, uint8_t msg_type);
void PD_PHY_TickMs(uint8_t delta_ms);
void PD_PHY_Poll(void);
void PD_PHY_FlushTxNow(void);
void PD_PHY_Abort(void);
void PD_PHY_ForceRxMode(void);
void PD_Analyzer_On_Valid_SrcCap(uint8_t ndo);
void PD_PHY_IRQHandler(void);

void PD_RX_Reserved(void);
void PD_RX_HRST(void);
void PD_RX_VDM_Reserved(void);
void Prot_NULL(void);

void PD_TX_HRST(void);
void PD_TX_BIST(void);
void PD_TX_NotSupported(void);
void PD_TX_Reject(void);
void PD_TX_ManufacturerInfo(void);
void PD_TX_Revision(void);
void PD_TX_SrcInfo(void);
void PD_TX_SinkCapExt(void);
void PD_TX_ChunkRequest(void);
void PD_TX_VCONN_DISC_IDENT(void);

#endif /* PD_PHY_H417_H_ */
