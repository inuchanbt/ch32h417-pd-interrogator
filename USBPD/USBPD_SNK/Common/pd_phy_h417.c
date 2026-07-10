/********************************** (C) COPYRIGHT *******************************
 * pd_phy_h417.c — PHY engine for PD_Prot on CH32H417 (replaces X035 lib + CH211 I2C).
 ********************************************************************************/

#include "pd_phy_h417.h"
#include "PD_Prot.h"
#include "PD_VDM.h"
#include "PD_Process.h"
#include "debug.h"
#include <string.h>

st_PD_PHY     PD_PHY;
st_PD_DEVICE  PD_DEVICE;

__attribute__((aligned(4))) uint16_t PD_RX_BUF[28];
__attribute__((aligned(4))) uint16_t PD_TX_BUF[28];
static __attribute__((aligned(4))) uint16_t s_irq_rx_buf[28];

#define PD_RX_DEFER_QUEUE_LEN 4u
static __attribute__((aligned(4))) uint16_t s_deferred_rx_queue[PD_RX_DEFER_QUEUE_LEN][28];
static uint8_t s_deferred_rx_sop[PD_RX_DEFER_QUEUE_LEN];

st_Prot_Header     *rxHeader    = (st_Prot_Header *)PD_RX_BUF;
st_Prot_Header     *txHeader    = (st_Prot_Header *)PD_TX_BUF;
st_Extended_Header *rxExtHeader = (st_Extended_Header *)&PD_RX_BUF[1];

uint16_t vSafe0V         = 151;
uint16_t vSinkDisconnect = 693;
uint16_t vVBUSon         = 100;
uint16_t GetADC_VBUS     = 500;

#define ANALYZER_ATTACH_DEBOUNCE_MS  8u
#define ANALYZER_GETSRCCAP_TIMEOUT_MS 1000u
#define ANALYZER_STALE_WAKE_MS      1600u
#define ANALYZER_STALE_BOUNCE_MS    1200u
#define ANALYZER_SOFTRESET_MS        350u
#define ANALYZER_SOFTRESET_RETRY_MS  450u
#define ANALYZER_SOFTRESET_MAX       2u
#define ANALYZER_HARDRESET_MS        450u
#define ANALYZER_HRST_RECOVERY_MS   1000u
#define ANALYZER_HRST_GIVEUP_MS      800u
#define ANALYZER_CABLE_BOUNCE_MS    1000u

static uint8_t s_analyzer_phy_started = 0;
static uint16_t s_analyzer_boot_ms    = 0;
static uint8_t s_analyzer_physical_cc = 0;   /* CC line selected, BMC listening */
static uint8_t s_analyzer_connected   = 0; /* protocol armed (pDevice_Attached) */
static uint8_t s_analyzer_active_cc   = 0;
static uint16_t s_analyzer_det_cnt    = 0;
static uint8_t s_analyzer_pending_cc  = 0;
static uint16_t s_analyzer_debounce_ms = 0;
static uint8_t s_analyzer_got_srccap  = 0;
static uint8_t s_analyzer_getsrc_sent = 0;
static uint8_t s_analyzer_cc_tried_mask = 0;
static uint16_t s_analyzer_srccap_wait_ms = 0;
static uint8_t s_analyzer_bounce_phase  = 0;
static uint16_t s_analyzer_bounce_ms    = 0;
static uint8_t s_analyzer_cable_at_boot = 0;
static uint8_t s_analyzer_wake_bounce   = 0;
static uint16_t s_analyzer_wake_ms      = 0;
static uint8_t s_analyzer_wake_done     = 0;
static uint8_t s_analyzer_wake_cycle    = 0;
static uint8_t s_analyzer_softreset_cnt = 0;
static uint8_t s_analyzer_hrst_sent       = 0;
static uint8_t s_analyzer_hrst_recovery   = 0;
static uint8_t s_analyzer_hrst_reinit_done = 0;
static uint16_t s_analyzer_hrst_recovery_ms = 0;

/* Defer pRxFinish until auto-GoodCRC TX completes (matches stock SNK Msg_Recvd flow). */
static volatile uint8_t s_waiting_gcrc_tx  = 0;
static volatile uint8_t s_defer_rx_finish = 0;
static volatile uint8_t s_deferred_rx_head = 0;
static volatile uint8_t s_deferred_rx_tail = 0;
static volatile uint8_t s_deferred_rx_count = 0;
/* Second RX that arrived while GoodCRC TX busy (HKY Cap during ACK GoodCRC). */
static volatile uint8_t s_pending_rx = 0;
static volatile uint8_t s_pending_rx_sop = 0;
static __attribute__((aligned(4))) uint16_t s_pending_rx_buf[28];

static void PD_PHY_FlushTx(void);
static void pd_defer_rx_packet(uint8_t rx_sop);
static void pd_clear_deferred_rx(void);
static void pd_resume_rx_preserve_latched(void);
static void pd_bytes_to_words(uint16_t *dst, const uint8_t *src, uint16_t bytes);
static void PD_GPIO_CC_HiZ(void);
static void PD_GPIO_USBPD_AF(void);
static void PD_Analyzer_Protocol_Attach(void);
extern void PD_VBUS_Update(void);
extern UINT8 PD_VBUS_Valid(void);

static void PD_Analyzer_Reset_Attach_State(void)
{
    s_analyzer_pending_cc     = 0;
    s_analyzer_debounce_ms    = 0;
    s_analyzer_got_srccap     = 0;
    s_analyzer_getsrc_sent    = 0;
    s_analyzer_srccap_wait_ms = 0;
}

/* Physical cable removal only — prints CC disconnect via pDevice_Unattached. */
static void PD_Analyzer_Do_Detach(void)
{
    if (!s_analyzer_physical_cc) {
        return;
    }
    s_analyzer_physical_cc = 0;
    s_analyzer_connected   = 0;
    s_analyzer_active_cc   = 0;
    s_analyzer_det_cnt     = 0;
    s_analyzer_cc_tried_mask = 0;
    s_analyzer_wake_cycle    = 0;
    s_analyzer_softreset_cnt = 0;
    s_analyzer_wake_done     = 0;
    s_analyzer_wake_bounce   = 0;
    s_analyzer_hrst_sent       = 0;
    s_analyzer_hrst_recovery   = 0;
    s_analyzer_hrst_reinit_done = 0;
    s_analyzer_hrst_recovery_ms = 0;
    PD_Analyzer_Reset_Attach_State();
    PD_Analyzer_Notify_Detach();
    pDevice_Unattached();
    PD_SINK_Init();
    PD_Rx_Mode();
}

static void PD_Analyzer_Protocol_Attach(void)
{
    if (PD_DEVICE.ConnectStat) {
        return;
    }
    s_analyzer_connected = 1;
    PD_Analyzer_Notify_Attach(s_analyzer_active_cc);
    pDevice_Attached();
}

/* Select CC + enable BMC; protocol stays off until valid SrcCap. */
static void PD_Analyzer_Physical_Attach(uint8_t cc)
{
    if (s_analyzer_physical_cc || cc == 0) {
        return;
    }
    s_analyzer_physical_cc    = cc;
    s_analyzer_active_cc        = cc;
    s_analyzer_connected        = 0;
    s_analyzer_det_cnt          = 0;
    s_analyzer_pending_cc       = 0;
    s_analyzer_debounce_ms      = 0;
    s_analyzer_got_srccap       = 0;
    s_analyzer_getsrc_sent      = 0;
    s_analyzer_srccap_wait_ms   = 0;
    s_analyzer_wake_done        = 0;
    s_analyzer_wake_cycle       = 0;
    s_analyzer_softreset_cnt    = 0;
    s_analyzer_hrst_sent          = 0;
    s_analyzer_hrst_recovery      = 0;
    s_analyzer_hrst_reinit_done   = 0;
    s_analyzer_hrst_recovery_ms   = 0;
    s_analyzer_cc_tried_mask   |= (uint8_t)(1u << (cc - 1u));
    if (cc == 2) {
        USBPD->CONFIG |= CC_SEL;
    } else {
        USBPD->CONFIG &= ~CC_SEL;
    }
    NVIC_EnableIRQ(USBPD_IRQn);
    PD_Rx_Mode();
    printf("CC%d SRC Connect\r\n", cc);
}

/* Wrong CC: swap line without pDevice_Unattached / CC disconnect message. */
static void PD_Analyzer_Silent_CC_Swap(uint8_t alt)
{
    if (!s_analyzer_physical_cc) {
        return;
    }
    if (PD_DEVICE.ConnectStat) {
        PD_PHY_Abort();
        NVIC_DisableIRQ(USBPD_IRQn);
        PD_PHY.WaitTxGcrc = PD_PHY.WaitRxGcrc = PD_PHY.WaitMsgTx = PD_PHY.WaitMsgRx = 0;
        PD_DEVICE.ConnectStat = 0;
        s_analyzer_connected = 0;
        PD_Analyzer_Notify_Detach();
    }
    s_analyzer_physical_cc = 0;
    s_analyzer_active_cc   = 0;
    s_waiting_gcrc_tx      = 0;
    pd_clear_deferred_rx();
    s_pending_rx           = 0;
    PD_Analyzer_Reset_Attach_State();
    PD_PHY_Reset_Value(0);
    PD_SINK_Init();
    printf("CC switch to CC%d\r\n", alt);
    PD_Analyzer_Physical_Attach(alt);
}

static void PD_Analyzer_Send_Get_SrcCap(void)
{
    if (!s_analyzer_physical_cc) {
        return;
    }
    PD_PHY_Header_Init(0, 0, PD_Ctrl_GetSrcCap);
    PD_PHY.MsgTxCnt = 0;
    PD_PHY_FlushTx();
    printf("TX Get_Source_Cap\r\n");
}

static uint8_t PD_Analyzer_Should_Get_SrcCap(void)
{
    PD_VBUS_Update();
    if (PD_VBUS_Valid()) {
        printf("VBUS valid, Get_Source_Cap\r\n");
        return 1;
    }
    if (s_analyzer_srccap_wait_ms >= ANALYZER_GETSRCCAP_TIMEOUT_MS) {
        printf("Get_Source_Cap timeout %ums\r\n", (unsigned)s_analyzer_srccap_wait_ms);
        return 1;
    }
    return 0;
}

static void PD_Analyzer_Send_SoftReset(void)
{
    if (!s_analyzer_physical_cc) {
        return;
    }
    PD_PHY.TxMsgID = 0;
    PD_PHY.RxMsgID = 0;
    /* cnt=0: Extended=0. cnt=1 would add a garbage extended word (CY4500 saw COUNTRY_INFO). */
    PD_PHY_Header_Init(0, 0, PD_Ctrl_SoftReset);
    PD_PHY.MsgTxCnt = 0;
    PD_PHY_FlushTx();
    printf("TX Soft_Reset\r\n");
}

static void PD_Analyzer_Send_HardReset(void)
{
    if (!s_analyzer_physical_cc || s_analyzer_hrst_sent) {
        return;
    }
    PD_PHY_Abort();
    PD_PHY.WaitTxGcrc = PD_PHY.WaitRxGcrc = PD_PHY.WaitMsgTx = PD_PHY.WaitMsgRx = 0;
    PD_PHY.TxMsgID = 0;
    PD_PHY.RxMsgID = 0;
    NVIC_DisableIRQ(USBPD_IRQn);
    PD_Phy_SendPack(0x01, NULL, 0, UPD_HARD_RESET);
    NVIC_EnableIRQ(USBPD_IRQn);
    PD_Rx_Mode();
    s_analyzer_hrst_sent        = 1;
    s_analyzer_hrst_recovery    = 1;
    s_analyzer_hrst_reinit_done = 0;
    s_analyzer_hrst_recovery_ms = 0;
    PD_Analyzer_Reset_Attach_State();
    printf("TX Hard_Reset\r\n");
}

static void PD_Analyzer_Hrst_Recovery_Tick(uint8_t delta_ms)
{
    if (!s_analyzer_hrst_recovery) {
        return;
    }
    if (s_analyzer_got_srccap) {
        s_analyzer_hrst_recovery = 0;
        return;
    }

    s_analyzer_hrst_recovery_ms = (uint16_t)(s_analyzer_hrst_recovery_ms + delta_ms);

    if (!s_analyzer_hrst_reinit_done &&
        s_analyzer_hrst_recovery_ms >= ANALYZER_HRST_RECOVERY_MS) {
        PD_GPIO_USBPD_AF();
        PD_SINK_Init();
        if (s_analyzer_active_cc == 2) {
            USBPD->CONFIG |= CC_SEL;
        } else {
            USBPD->CONFIG &= ~CC_SEL;
        }
        USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END;
        NVIC_EnableIRQ(USBPD_IRQn);
        PD_PHY_Reset_Value(0);
        PD_Rx_Mode();
        s_analyzer_hrst_reinit_done = 1;
        PD_Analyzer_Reset_Attach_State();
        printf("CC%d HRST recovery done\r\n", s_analyzer_active_cc);
    }

    if (s_analyzer_hrst_reinit_done &&
        !s_analyzer_getsrc_sent &&
        s_analyzer_hrst_recovery_ms >= (uint16_t)(ANALYZER_HRST_RECOVERY_MS + ANALYZER_GETSRCCAP_TIMEOUT_MS)) {
        s_analyzer_getsrc_sent = 1;
        PD_Analyzer_Send_Get_SrcCap();
    }

    if (s_analyzer_hrst_reinit_done &&
        s_analyzer_getsrc_sent &&
        s_analyzer_hrst_recovery_ms >= (uint16_t)(ANALYZER_HRST_RECOVERY_MS + ANALYZER_GETSRCCAP_TIMEOUT_MS +
                                                  ANALYZER_HRST_GIVEUP_MS)) {
        s_analyzer_hrst_recovery = 0;
    }
}

static uint8_t pd_analyzer_preattach_ctrl(uint8_t msg_type)
{
    return (msg_type == PD_Ctrl_Accept || msg_type == PD_Ctrl_SoftReset);
}

static void PD_Analyzer_Stale_Wake_Start(void)
{
    if (!s_analyzer_physical_cc || s_analyzer_wake_bounce) {
        return;
    }
    s_analyzer_wake_bounce = 1;
    s_analyzer_wake_ms     = 0;
    s_analyzer_det_cnt     = 0;
    PD_GPIO_CC_HiZ();
    printf("CC%d SrcCap stale, Hi-Z wake\r\n", s_analyzer_active_cc);
}

static void PD_Analyzer_Srccap_Tick(uint8_t delta_ms)
{
    if (!s_analyzer_physical_cc || s_analyzer_got_srccap) {
        return;
    }

    if (s_analyzer_hrst_recovery) {
        PD_Analyzer_Hrst_Recovery_Tick(delta_ms);
        return;
    }

    if (s_analyzer_wake_bounce) {
        s_analyzer_wake_ms = (uint16_t)(s_analyzer_wake_ms + delta_ms);
        if (s_analyzer_wake_ms < ANALYZER_STALE_BOUNCE_MS) {
            return;
        }
        s_analyzer_wake_bounce = 0;
        PD_GPIO_USBPD_AF();
        PD_SINK_Init();
        if (s_analyzer_active_cc == 2) {
            USBPD->CONFIG |= CC_SEL;
        } else {
            USBPD->CONFIG &= ~CC_SEL;
        }
        NVIC_EnableIRQ(USBPD_IRQn);
        PD_Rx_Mode();
        PD_Analyzer_Reset_Attach_State();
        s_analyzer_wake_cycle = 1;
        printf("CC%d wake done\r\n", s_analyzer_active_cc);
        return;
    }

    s_analyzer_srccap_wait_ms = (uint16_t)(s_analyzer_srccap_wait_ms + delta_ms);

    if (!s_analyzer_getsrc_sent && PD_Analyzer_Should_Get_SrcCap()) {
        s_analyzer_getsrc_sent = 1;
        PD_Analyzer_Send_Get_SrcCap();
    }

    if (s_analyzer_wake_cycle >= 1u && !s_analyzer_got_srccap && !s_analyzer_hrst_sent) {
        if (s_analyzer_softreset_cnt == 0u &&
            s_analyzer_srccap_wait_ms >= ANALYZER_SOFTRESET_MS) {
            s_analyzer_softreset_cnt = 1;
            s_analyzer_wake_cycle    = 2;
            PD_Analyzer_Send_SoftReset();
            PD_Analyzer_Reset_Attach_State();
            return;
        }
        if (s_analyzer_softreset_cnt == 1u &&
            s_analyzer_srccap_wait_ms >= ANALYZER_SOFTRESET_RETRY_MS) {
            s_analyzer_softreset_cnt = 2;
            PD_Analyzer_Send_SoftReset();
            PD_Analyzer_Reset_Attach_State();
            return;
        }
        if (s_analyzer_softreset_cnt >= ANALYZER_SOFTRESET_MAX &&
            s_analyzer_srccap_wait_ms >= ANALYZER_HARDRESET_MS) {
            PD_Analyzer_Send_HardReset();
            return;
        }
    }

    if (s_analyzer_wake_cycle == 0u && !s_analyzer_wake_done && !s_analyzer_hrst_sent &&
        s_analyzer_srccap_wait_ms >= ANALYZER_STALE_WAKE_MS) {
        s_analyzer_wake_done = 1;
        PD_Analyzer_Stale_Wake_Start();
    }
}

void PD_Analyzer_On_Valid_SrcCap(uint8_t ndo)
{
    if (ndo > 0u) {
        s_analyzer_got_srccap       = 1;
        s_analyzer_cc_tried_mask    = 0;
        s_analyzer_srccap_wait_ms   = 0;
    }
}

static void PD_Analyzer_Attach_Tick(uint8_t delta_ms)
{
    uint8_t status;

    if (!s_analyzer_phy_started) {
        return;
    }

    if (s_analyzer_bounce_phase) {
        s_analyzer_bounce_ms = (uint16_t)(s_analyzer_bounce_ms + delta_ms);
        if (s_analyzer_bounce_ms < ANALYZER_CABLE_BOUNCE_MS) {
            return;
        }
        s_analyzer_bounce_phase = 0;
        PD_GPIO_USBPD_AF();
        PD_SINK_Init();
        PD_Rx_Mode();
        printf("CC bounce done\r\n");
        if (s_analyzer_cable_at_boot) {
            s_analyzer_cable_at_boot = 0;
            status = PD_Detect_Analyzer(0, 0);
            if (status != 0) {
                PD_Analyzer_Physical_Attach(status);
            }
            return;
        }
    }

    if (s_analyzer_physical_cc) {
        return;
    }

    status = PD_Detect_Analyzer(0, 0);
    if (status == 0) {
        s_analyzer_pending_cc  = 0;
        s_analyzer_debounce_ms = 0;
        return;
    }
    if (status != s_analyzer_pending_cc) {
        s_analyzer_pending_cc  = status;
        s_analyzer_debounce_ms = delta_ms;
        return;
    }
    s_analyzer_debounce_ms = (uint16_t)(s_analyzer_debounce_ms + delta_ms);
    if (s_analyzer_debounce_ms < ANALYZER_ATTACH_DEBOUNCE_MS) {
        return;
    }
    PD_Analyzer_Physical_Attach(status);
}

static void pd_words_to_bytes(uint8_t *dst, const uint16_t *src, uint16_t words)
{
    uint16_t i;
    for (i = 0; i < words; i++) {
        dst[i * 2u]     = (uint8_t)(src[i] & 0xFFu);
        dst[i * 2u + 1] = (uint8_t)(src[i] >> 8);
    }
}

static void pd_bytes_to_words(uint16_t *dst, const uint8_t *src, uint16_t bytes)
{
    uint16_t i;
    uint16_t words = (uint16_t)((bytes + 1u) / 2u);
    for (i = 0; i < words; i++) {
        dst[i] = (uint16_t)src[i * 2u] | ((uint16_t)src[i * 2u + 1] << 8);
    }
}

static uint8_t pd_phy_rx_sop(uint16_t status)
{
    switch (status & BMC_AUX_Mask) {
    case BMC_AUX_SOP0:
        return PD_PHY_RX_SOP;
    case BMC_AUX_SOP1_HRST:
        return PD_PHY_RX_SOPP;
    case BMC_AUX_SOP2_CRST:
        return PD_PHY_RX_SOPPP;
    default:
        return 0;
    }
}

static uint8_t pd_phy_auto_goodcrc(uint8_t rx_sop)
{
    if (rx_sop == PD_PHY_RX_SOPP && PD_PHY.ListenOnlySopp) {
        return 0;
    }
    if (PD_PHY.RxSop != 0 && rx_sop != PD_PHY.RxSop) {
        return 0;
    }
    return 1;
}

static uint8_t pd_analyzer_may_goodcrc(uint8_t rx_sop)
{
    if (!pd_phy_auto_goodcrc(rx_sop)) {
        return 0;
    }
    if (PD_DEVICE.ConnectStat) {
        return 1;
    }
    if (s_analyzer_physical_cc != 0 &&
        ((PD_Rx_Buf[0] & 0x1Fu) == PD_Data_SrcCap ||
         pd_analyzer_preattach_ctrl((uint8_t)(PD_Rx_Buf[0] & 0x1Fu)))) {
        return 1;
    }
    return 0;
}

static uint8_t PD_PHY_DoTxNow(void)
{
    uint8_t  ndo;
    uint16_t words;
    uint8_t  byte_len;
    uint8_t  sop;
    uint16_t wait;
    uint8_t  got_crc = 0;

    ndo = txHeader->NDO;
    /*
     * NDO already counts every 32-bit object after the Message Header.
     * For Extended messages the Extended Header lives inside object 0;
     * it must not add another 16-bit word here.
     */
    words = (uint16_t)(1u + ndo * 2u);
    byte_len = (uint8_t)(words * 2u);
    if (byte_len > sizeof(PD_Tx_Buf) ||
        words > (sizeof(PD_TX_BUF) / sizeof(PD_TX_BUF[0]))) {
        return 0;
    }

    pd_words_to_bytes(PD_Tx_Buf, PD_TX_BUF, words);
    sop = PD_PHY.TxSop ? PD_PHY.TxSop : UPD_SOP0;

    /*
     * Single-shot TX.  Never retry — same-MsgID retries cause SoftReset storms
     * (anker_076-2).  Never drop non-GoodCRC RX while polling — that ate EPR
     * Mode ACK / SrcCap and led to HardReset (battery_076-2, aohi-240w_076).
     *
     * aohi-240w/ugreen 083: EPR Mode ACK arrived ~2ms after Enter GoodCRC but
     * sink never GoodCRCd it (3× retry → "Enter: no response").  Stale
     * s_waiting_gcrc_tx made IRQ stash ACK without ACKing; also PD_Rx_Mode's
     * PD_ALL_CLR right after Enter GoodCRC raced the reply.  Clear TX-wait
     * state, then soft-poll for the reply before enabling IRQ.
     */
    NVIC_DisableIRQ(USBPD_IRQn);
    s_waiting_gcrc_tx = 0;
    s_pending_rx      = 0;
    USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
    USBPD->STATUS = 0xFFu;
    PD_Phy_SendPack(0x01, PD_Tx_Buf, byte_len, sop);

    wait = 400; /* ~1.2ms */
    while (--wait) {
        if ((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT) {
            USBPD->STATUS |= IF_RX_ACT;
            /* GoodCRC is always 6B; SrcCap also has MsgType=0x01 — use length. */
            if ((USBPD->BMC_BYTE_CNT == 6u) &&
                ((PD_Rx_Buf[0] & 0x1Fu) == DEF_TYPE_GOODCRC) &&
                ((PD_Rx_Buf[1] & 0x0Eu) == (PD_Tx_Buf[1] & 0x0Eu))) {
                got_crc = 1;
                break; /* normal GoodCRC */
            }
            if ((USBPD->BMC_BYTE_CNT > 6u) ||
                ((USBPD->BMC_BYTE_CNT == 6u) &&
                 ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC))) {
                /* Source already replied (Accept / EPR_Mode / SrcCap). */
                goto reply_inline_ack;
            }
        }
        Delay_Us(3);
    }

    /* Soft re-arm — no PD_ALL_CLR (would wipe a reply in flight). */
    USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END | PD_DMA_EN;
    USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
    USBPD->CONTROL |= BMC_START;

    /* ~5ms window for EPR Mode ACK / Accept / Cap after our GoodCRC. */
    wait = 1700;
    while (--wait) {
        if ((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT) {
            USBPD->STATUS |= IF_RX_ACT;
            if ((USBPD->BMC_BYTE_CNT > 6u) ||
                ((USBPD->BMC_BYTE_CNT == 6u) &&
                 ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC))) {
                goto reply_inline_ack;
            }
            if ((USBPD->BMC_BYTE_CNT == 6u) &&
                ((PD_Rx_Buf[0] & 0x1Fu) == DEF_TYPE_GOODCRC) && !got_crc) {
                got_crc = 1;
                USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
                USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END | PD_DMA_EN;
                USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
                USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
                USBPD->CONTROL |= BMC_START;
            }
        }
        Delay_Us(3);
    }

    PD_PHY.TxMsgID = (uint8_t)((PD_PHY.TxMsgID + 1u) & 0x07u);
    PD_Rx_Mode();
    NVIC_EnableIRQ(USBPD_IRQn);
    return 1;

reply_inline_ack:
    pd_bytes_to_words(s_irq_rx_buf, PD_Rx_Buf, USBPD->BMC_BYTE_CNT);
    pd_defer_rx_packet(PD_PHY_RX_SOP);
    PD_PHY.TxMsgID = (uint8_t)((PD_PHY.TxMsgID + 1u) & 0x07u);
    Delay_Us(30);
    PD_Ack_Buf[0] = (uint8_t)(DEF_TYPE_GOODCRC | (PD_Rx_Buf[0] & 0xC0u));
    PD_Ack_Buf[1] = (PD_Rx_Buf[1] & 0x0Eu);
    PD_Phy_SendPack(0x01, PD_Ack_Buf, 2, UPD_SOP0);
    NVIC_EnableIRQ(USBPD_IRQn);
    return 1;
}

void PD_PHY_H417_Init(void)
{
    memset(&PD_PHY, 0, sizeof(PD_PHY));
    memset(&PD_DEVICE, 0, sizeof(PD_DEVICE));
    PD_PHY.RxSop = PD_PHY_RX_SOP;
    PD_PHY.LastRxSop = 0;
    PD_PHY.TxSop = PD_PHY_TX_SOP;
    PD_PHY.Header.SpecRevision = PD_Rev2;
    PD_PHY.Header.PortPwrRole  = 0;
    PD_PHY.Header.PortDataRole = 0;
}

void Set_DevChk(uint8_t role)
{
    PD_DEVICE.DevRole = role;
    PD_DEVICE.DevStat = (role == DevRole_Src) ? 1u : 0u;
}

void PD_PHY_Reset_Value(uint8_t role)
{
    (void)role;
    PD_PHY.TxMsgID = 0;
    PD_PHY.RxMsgID = 0;
    PD_PHY.RetryCnt = 0;
    PD_PHY.PDExist = 0;
    PD_PHY.WaitTxGcrc = 0;
    PD_PHY.WaitRxGcrc = 0;
    PD_PHY.WaitMsgTx = 0;
    PD_PHY.WaitMsgRx = 0;
    PD_PHY.MsgTxCnt = 0;
    PD_PHY.MsgRxCnt = 0;
    PD_PHY.RxSop = PD_PHY_RX_SOP;
    PD_PHY.LastRxSop = 0;
    PD_PHY.TxSop = PD_PHY_TX_SOP;
    PD_PHY.ListenOnlySopp = 0;
    s_waiting_gcrc_tx   = 0;
    pd_clear_deferred_rx();
    s_pending_rx        = 0;
}

void PD_Prot_pSet(void (*pTxFinish)(void), void (*pRxFinish)(void),
                  void (*pTxTimeout)(void), void (*pRxTimeout)(void))
{
    PD_PHY.pTxFinish  = pTxFinish;
    PD_PHY.pRxFinish  = pRxFinish;
    PD_PHY.pTxTimeout = pTxTimeout;
    PD_PHY.pRxTimeout = pRxTimeout;
}

void PD_PHY_Set_RxSop(uint8_t sop)
{
    PD_PHY.RxSop = sop;
}

void PD_PHY_Set_ListenOnlySopp(uint8_t on)
{
    PD_PHY.ListenOnlySopp = on ? 1u : 0u;
}

void PD_PHY_Header_Init(uint8_t cnt, uint8_t ndo, uint8_t msg_type)
{
    (void)cnt;
    txHeader->MsgType       = msg_type;
    txHeader->NDO           = ndo;
    txHeader->MsgID         = PD_PHY.TxMsgID & 0x07u;
    txHeader->PortDataRole  = PD_PHY.Header.PortDataRole;
    txHeader->PortPwrRole   = PD_PHY.Header.PortPwrRole;
    txHeader->SpecRevision  = PD_PHY.Header.SpecRevision;
    txHeader->Extended      = 0;
    PD_PHY.WaitMsgTx = 1;
    PD_PHY.MsgTxCnt  = 0;
    printf("tx");
}

static void PD_PHY_SendPending(void)
{
    uint8_t ok;

    if (!PD_PHY.WaitMsgTx) {
        return;
    }
    PD_PHY.WaitMsgTx = 0;
    ok = PD_PHY_DoTxNow();
    if (ok) {
        if (PD_PHY.pTxFinish) {
            void (*fn)(void) = PD_PHY.pTxFinish;
            PD_PHY.pTxFinish = NULL;
            fn();
        }
    } else if (PD_PHY.pTxTimeout) {
        void (*fn)(void) = PD_PHY.pTxTimeout;
        PD_PHY.pTxTimeout = NULL;
        fn();
    }
}

static void PD_PHY_FlushTx(void)
{
    if (!PD_PHY.WaitMsgTx || PD_PHY.MsgTxCnt > 0) {
        return;
    }
    PD_PHY_SendPending();
}

static void pd_defer_rx_packet(uint8_t rx_sop)
{
    uint8_t tail;

    if (s_deferred_rx_count >= PD_RX_DEFER_QUEUE_LEN) {
        return;
    }

    tail = s_deferred_rx_tail;
    memcpy(s_deferred_rx_queue[tail], s_irq_rx_buf,
           sizeof(s_deferred_rx_queue[tail]));
    s_deferred_rx_sop[tail] = rx_sop;
    s_deferred_rx_tail = (uint8_t)((tail + 1u) % PD_RX_DEFER_QUEUE_LEN);
    s_deferred_rx_count++;
    s_defer_rx_finish = 1;
}

static void pd_clear_deferred_rx(void)
{
    s_defer_rx_finish = 0;
    s_deferred_rx_head = 0;
    s_deferred_rx_tail = 0;
    s_deferred_rx_count = 0;
}

static void pd_resume_rx_preserve_latched(void)
{
    /*
     * TX_END already re-arms BMC before deferring the packet.  If another
     * packet completed while NVIC was paused, PD_Rx_Mode() would assert
     * PD_ALL_CLR and erase that latched IF_RX_ACT.  Only touch BMC when no
     * complete packet is waiting, then let the pending IRQ run immediately.
     */
    if ((USBPD->STATUS & IF_RX_ACT) == 0u) {
        USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
        USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END | PD_DMA_EN;
        USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
        USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
        USBPD->CONTROL |= BMC_START;
    }
    NVIC_EnableIRQ(USBPD_IRQn);
}

void PD_PHY_FlushTxNow(void)
{
    PD_PHY.MsgTxCnt = 0;
    PD_PHY_FlushTx();
}

void PD_PHY_TickMs(uint8_t delta_ms)
{
    if (PD_PHY.WaitMsgTx && PD_PHY.MsgTxCnt > 0) {
        if (PD_PHY.MsgTxCnt <= delta_ms) {
            PD_PHY.MsgTxCnt = 0;
            PD_PHY_FlushTx();
        } else {
            PD_PHY.MsgTxCnt = (uint16_t)(PD_PHY.MsgTxCnt - delta_ms);
        }
    }

    if (PD_PHY.WaitMsgRx && PD_PHY.MsgRxCnt > 0) {
        if (PD_PHY.MsgRxCnt > delta_ms) {
            PD_PHY.MsgRxCnt = (uint16_t)(PD_PHY.MsgRxCnt - delta_ms);
        } else {
            PD_PHY.MsgRxCnt = 0;
            PD_PHY.WaitMsgRx = 0;
            if (PD_PHY.pRxTimeout) {
                void (*fn)(void) = PD_PHY.pRxTimeout;
                PD_PHY.pRxTimeout = NULL;
                fn();
            }
        }
    }

    PD_Analyzer_Attach_Tick(delta_ms);
    PD_Analyzer_Srccap_Tick(delta_ms);
}

static void PD_PHY_RunDeferredRx(void)
{
    void (*fn)(void) = NULL;
    uint8_t rx_sop;
    uint8_t head;
    uint8_t drain = 0;

    /*
     * Drain nested packets that arrived while a callback was printing.
     * aohi-240w: Cap lands during "Enter Succeeded" printf; without a drain
     * loop the Cap sits until the next Poll and SoftReset/HRST wins the race.
     */
    while (s_defer_rx_finish) {
        /* Never abort an in-flight software GoodCRC to run a callback. */
        if ((USBPD->CONTROL & PD_TX_EN) != 0u) {
            return;
        }

        NVIC_DisableIRQ(USBPD_IRQn);
        if (s_deferred_rx_count == 0u) {
            s_defer_rx_finish = 0;
            NVIC_EnableIRQ(USBPD_IRQn);
            break;
        }

        head = s_deferred_rx_head;
        rx_sop = s_deferred_rx_sop[head];
        memcpy(PD_RX_BUF, s_deferred_rx_queue[head], sizeof(PD_RX_BUF));
        s_deferred_rx_head = (uint8_t)((head + 1u) % PD_RX_DEFER_QUEUE_LEN);
        s_deferred_rx_count--;
        s_defer_rx_finish = (s_deferred_rx_count != 0u);

        fn = NULL;
        if (rx_sop == PD_PHY_RX_SOP && PD_PHY.pRxFinish) {
            fn = PD_PHY.pRxFinish;
            PD_PHY.pRxFinish = NULL;
        }

        if (rx_sop != 0 && rx_sop != PD_PHY_RX_SOP) {
            PD_Rx_Mode();
            VDM_Sniff_SOPP_Cable();
            continue;
        }

        /* Resume SOP RX without clearing a packet that arrived while paused. */
        pd_resume_rx_preserve_latched();

        if (fn) {
            fn();
        } else if (s_analyzer_physical_cc && rxHeader->MsgType == PD_Data_SrcCap &&
                   rxHeader->NDO > 0u) {
            if (!PD_DEVICE.ConnectStat) {
                PD_Analyzer_Protocol_Attach();
            }
            pProt_RX_SrcCap();
        } else if (s_analyzer_physical_cc && !PD_DEVICE.ConnectStat &&
                   rxHeader->MsgType == PD_Ctrl_SoftReset) {
            PD_PHY.TxMsgID = 0;
            PD_PHY.RxMsgID = 0;
            PD_PHY_Header_Init(0, 0, PD_Ctrl_Accept);
            PD_PHY.MsgTxCnt = 0;
            PD_PHY_FlushTx();
            PD_Analyzer_Reset_Attach_State();
        } else if (s_analyzer_physical_cc && !PD_DEVICE.ConnectStat &&
                   rxHeader->MsgType == PD_Ctrl_Accept) {
            PD_PHY.TxMsgID = 0;
            PD_PHY.RxMsgID = 0;
            PD_Analyzer_Reset_Attach_State();
        }

        if (PD_PHY.WaitMsgTx) {
            PD_PHY_FlushTx();
        }

        if (++drain > 8u) {
            break; /* safety */
        }
    }
}

void PD_PHY_Poll(void)
{
    PD_PHY_RunDeferredRx();
}

void PD_PHY_Abort(void)
{
    s_waiting_gcrc_tx   = 0;
    pd_clear_deferred_rx();
    s_pending_rx        = 0;
    PD_PHY_ForceRxMode();
}

void PD_PHY_ForceRxMode(void)
{
    USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
    USBPD->STATUS = 0xFFu;
    USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
    USBPD->BMC_BYTE_CNT = 0;
    USBPD->BMC_TX_SZ = 0;
    USBPD->CONTROL |= BMC_START;
}

void PD_PHY_IRQHandler(void)
{
    uint8_t rx_sop;

    if (USBPD->STATUS & IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;
        rx_sop = pd_phy_rx_sop(USBPD->STATUS);
        PD_PHY.LastRxSop = rx_sop;
        if (rx_sop == PD_PHY_RX_SOP) {
            if (USBPD->BMC_BYTE_CNT >= 6) {
                /*
                 * GoodCRC and Source_Capabilities share MsgType=0x01.
                 * Distinguish by wire length: GoodCRC is header+CRC only (6B).
                 * NEVER drop on MsgType alone — that silenced all SrcCap (077).
                 */
                if ((USBPD->BMC_BYTE_CNT != 6) ||
                    ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC)) {
                    if (s_waiting_gcrc_tx) {
                        /*
                         * GoodCRC TX in progress (e.g. EPR Mode ACK).  HKY sends
                         * unchunked EPR Cap immediately after; overwriting
                         * s_irq_rx_buf and skipping GoodCRC caused SoftReset storms.
                         * Stash Cap and GoodCRC it right after current TX_END.
                         */
                        pd_bytes_to_words(s_pending_rx_buf, PD_Rx_Buf, USBPD->BMC_BYTE_CNT);
                        s_pending_rx_sop = rx_sop;
                        s_pending_rx = 1;
                    } else {
                        pd_bytes_to_words(s_irq_rx_buf, PD_Rx_Buf, USBPD->BMC_BYTE_CNT);
                        PD_PHY.WaitMsgRx = 0;
                        PD_PHY.MsgRxCnt  = 0;
                        if (pd_analyzer_may_goodcrc(rx_sop)) {
                            Delay_Us(30);
                            /*
                             * GoodCRC must echo the received Message ID and use the
                             * negotiated SpecRevision.  The old fixed 0x41 encoded
                             * PD 2.0, so PD 3.x sources ignored it and retransmitted
                             * every message until the PHY state became desynchronized.
                             * DataRole/PowerRole are both Sink/UFP (zero) here.
                             */
                            PD_Ack_Buf[0] = (uint8_t)(DEF_TYPE_GOODCRC |
                                                      (PD_Rx_Buf[0] & 0xC0u));
                            PD_Ack_Buf[1] = (PD_Rx_Buf[1] & 0x0Eu);
                            USBPD->CONFIG |= IE_TX_END;
                            PD_Phy_SendPack(0, PD_Ack_Buf, 2, UPD_SOP0);
                            s_waiting_gcrc_tx = 1;
                        } else if (!pd_phy_auto_goodcrc(rx_sop)) {
                            NVIC_DisableIRQ(USBPD_IRQn);
                            pd_defer_rx_packet(rx_sop);
                        }
                    }
                }
            }
        } else if (rx_sop == PD_PHY_RX_SOPP) {
            if (USBPD->BMC_BYTE_CNT >= 6) {
                if ((USBPD->BMC_BYTE_CNT != 6) ||
                    ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC)) {
                    pd_bytes_to_words(s_irq_rx_buf, PD_Rx_Buf, USBPD->BMC_BYTE_CNT);
                    NVIC_DisableIRQ(USBPD_IRQn);
                    pd_defer_rx_packet(rx_sop);
                }
            }
        } else if (rx_sop == PD_PHY_RX_SOPPP) {
            if (USBPD->BMC_BYTE_CNT >= 6) {
                if ((USBPD->BMC_BYTE_CNT != 6) ||
                    ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC)) {
                    pd_bytes_to_words(s_irq_rx_buf, PD_Rx_Buf, USBPD->BMC_BYTE_CNT);
                    NVIC_DisableIRQ(USBPD_IRQn);
                    pd_defer_rx_packet(rx_sop);
                }
            }
        }
    }

    if (USBPD->STATUS & IF_TX_END) {
        USBPD->PORT_CC1 &= ~CC_LVE;
        USBPD->PORT_CC2 &= ~CC_LVE;
        USBPD->STATUS |= IF_TX_END;

        if (s_waiting_gcrc_tx) {
            s_waiting_gcrc_tx  = 0;
            pd_defer_rx_packet(PD_PHY.LastRxSop);
            if (s_pending_rx) {
                /* GoodCRC the stashed Cap/message that arrived during prior GoodCRC TX. */
                uint8_t *pb = (uint8_t *)s_pending_rx_buf;
                s_pending_rx = 0;
                memcpy(s_irq_rx_buf, s_pending_rx_buf, sizeof(s_irq_rx_buf));
                PD_PHY.LastRxSop = s_pending_rx_sop;
                PD_Ack_Buf[0] = (uint8_t)(DEF_TYPE_GOODCRC | (pb[0] & 0xC0u));
                PD_Ack_Buf[1] = (pb[1] & 0x0Eu);
                USBPD->CONFIG |= IE_TX_END;
                if ((USBPD->CONFIG & CC_SEL) == CC_SEL) {
                    USBPD->PORT_CC2 |= CC_LVE;
                } else {
                    USBPD->PORT_CC1 |= CC_LVE;
                }
                PD_Phy_SendPack(0, PD_Ack_Buf, 2, UPD_SOP0);
                s_waiting_gcrc_tx = 1;
                /* Leave IRQ disabled until this second GoodCRC completes. */
            } else {
                /*
                 * Re-arm RX, then check IF_RX_ACT before disabling IRQ.
                 * HKY/aohi-240w send EPR Cap within ~1ms of Mode ACK GoodCRC;
                 * leaving IRQ off until Poll() dropped that Cap (080 SoftReset storm).
                 */
                USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
                USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END | PD_DMA_EN;
                USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
                USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
                USBPD->CONTROL |= BMC_START;

                if ((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT) {
                    USBPD->STATUS |= IF_RX_ACT;
                    if ((USBPD->BMC_BYTE_CNT >= 6u) &&
                        ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC)) {
                        pd_bytes_to_words(s_pending_rx_buf, PD_Rx_Buf,
                                          USBPD->BMC_BYTE_CNT);
                        s_pending_rx_sop = PD_PHY_RX_SOP;
                        s_pending_rx = 1;
                        PD_Ack_Buf[0] = (uint8_t)(DEF_TYPE_GOODCRC |
                                                  (PD_Rx_Buf[0] & 0xC0u));
                        PD_Ack_Buf[1] = (PD_Rx_Buf[1] & 0x0Eu);
                        USBPD->CONFIG |= IE_TX_END;
                        if ((USBPD->CONFIG & CC_SEL) == CC_SEL) {
                            USBPD->PORT_CC2 |= CC_LVE;
                        } else {
                            USBPD->PORT_CC1 |= CC_LVE;
                        }
                        PD_Phy_SendPack(0, PD_Ack_Buf, 2, UPD_SOP0);
                        s_waiting_gcrc_tx = 1;
                        return;
                    }
                }
                /* Safe to pause IRQ until deferred Mode/Accept is processed. */
                NVIC_DisableIRQ(USBPD_IRQn);
            }
        } else {
            PD_Rx_Mode();
        }
    }

    if (USBPD->STATUS & IF_RX_RESET) {
        USBPD->STATUS |= IF_RX_RESET;
        s_waiting_gcrc_tx = 0;
        pd_clear_deferred_rx();
        s_pending_rx = 0;
        if (PD_DEVICE.ConnectStat) {
            pProt_Wait_SrcCap();
        } else if (s_analyzer_physical_cc && s_analyzer_hrst_recovery) {
            PD_PHY.TxMsgID = 0;
            PD_PHY.RxMsgID = 0;
            if (s_analyzer_hrst_recovery_ms < (ANALYZER_HRST_RECOVERY_MS - 300u)) {
                s_analyzer_hrst_recovery_ms = (uint16_t)(ANALYZER_HRST_RECOVERY_MS - 300u);
            }
        }
        PD_Rx_Mode();
    }
}

void PD_RX_Reserved(void) { }
void PD_RX_VDM_Reserved(void) { }
void Prot_NULL(void) { }

void PD_RX_HRST(void)
{
    PD_TX_HRST();
}

static void PD_PHY_SendCtrl(uint8_t msg)
{
    PD_PHY_Header_Init(1, 0, msg);
    PD_PHY.MsgTxCnt = 0;
    PD_PHY_FlushTx();
}

void PD_TX_Reject(void)         { PD_PHY_SendCtrl(PD_Ctrl_Reject); }
void PD_TX_NotSupported(void)   { PD_PHY_SendCtrl(PD_Ctrl_NotSupported); }
void PD_TX_BIST(void)           { PD_PHY_SendCtrl(PD_Ctrl_Reject); }

void PD_TX_Revision(void)
{
    PD_PHY_Header_Init(1, 1, PD_Data_Revision);
    PD_TX_BUF[1] = 0x17AC;
    PD_TX_BUF[2] = 0x0000;
    PD_PHY.MsgTxCnt = 1;
    PD_PHY_FlushTx();
}

void PD_TX_SrcInfo(void)
{
    PD_TX_NotSupported();
}

void PD_TX_SinkCapExt(void)
{
    PD_PHY_Header_Init(1, 1, PD_Ctrl_NotSupported);
    PD_PHY.MsgTxCnt = 1;
    PD_PHY_FlushTx();
}

void PD_TX_ManufacturerInfo(void)
{
    PD_TX_NotSupported();
}

void PD_TX_ChunkRequest(void)
{
    uint8_t  next_chunk = (uint8_t)((rxExtHeader->ChunkNumber + 1u) & 0x0Fu);
    uint16_t wait;
    uint8_t  got_crc = 0;

    /*
     * Extended Chunk Request (single-shot).
     * Do not call PD_PHY_Header_Init — its printf("tx") delays before BMC TX.
     *
     * HKY captive/detachable (081): chunk1 is on the wire ~2–4ms after our
     * Chunk-Request GoodCRC, but the sink never GoodCRCs it (3× retry → HRST,
     * Object 8+ empty). Anker chunk1 is ~5.7ms. IRQ-only listen after
     * PD_Rx_Mode was not enough for HKY — poll chunk1 here with IRQ off,
     * re-arm BMC after the Chunk-Request GoodCRC (SendPack leaves RX armed
     * only until that GoodCRC is consumed).
     */
    txHeader->MsgType       = rxHeader->MsgType;
    txHeader->NDO           = 1;
    txHeader->MsgID         = PD_PHY.TxMsgID & 0x07u;
    txHeader->PortDataRole  = PD_PHY.Header.PortDataRole;
    txHeader->PortPwrRole   = PD_PHY.Header.PortPwrRole;
    txHeader->SpecRevision  = PD_PHY.Header.SpecRevision;
    txHeader->Extended      = 1;
    PD_TX_BUF[1] = (uint16_t)(0x8400u | ((uint16_t)next_chunk << 11));
    PD_TX_BUF[2] = 0;
    PD_PHY.WaitMsgTx = 0;
    PD_PHY.MsgTxCnt  = 0;

    pd_words_to_bytes(PD_Tx_Buf, PD_TX_BUF, 3u); /* hdr + 1 DO */

    NVIC_DisableIRQ(USBPD_IRQn);
    USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
    USBPD->STATUS = 0xFFu;
    PD_Phy_SendPack(0x01, PD_Tx_Buf, 6, UPD_SOP0);

    wait = 400; /* ~1.2ms GoodCRC window */
    while (--wait) {
        if ((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT) {
            USBPD->STATUS |= IF_RX_ACT;
            if ((USBPD->BMC_BYTE_CNT == 6u) &&
                ((PD_Rx_Buf[0] & 0x1Fu) == DEF_TYPE_GOODCRC)) {
                PD_PHY.TxMsgID = (uint8_t)((PD_PHY.TxMsgID + 1u) & 0x07u);
                got_crc = 1;
                break;
            }
            if (USBPD->BMC_BYTE_CNT > 6u) {
                /* Chunk1 before Chunk-Request GoodCRC — keep + GoodCRC it. */
                goto chunk1_inline_ack;
            }
        }
        Delay_Us(3);
    }

    /*
     * Re-arm BMC after GoodCRC without STATUS=0xFF wipe.
     * aohi-battery (082): chunk1 follows GoodCRC by ~180us — clearing STATUS
     * here discarded it; retries then also missed while IRQ stayed off.
     */
    USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END | PD_DMA_EN;
    USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
    USBPD->CONTROL |= BMC_START;

    /* Peek once before the long poll — catch ultra-fast chunk1. */
    if ((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;
        if ((USBPD->BMC_BYTE_CNT > 6u) &&
            ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC)) {
            goto chunk1_inline_ack;
        }
    }

    /*
     * Block for chunk1: HKY ~2–4ms, Anker ~5.7ms, aohi-battery ~0.2ms.
     * ~8ms covers the useful window including one retry.
     */
    wait = 2700;
    while (--wait) {
        if ((USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT) {
            USBPD->STATUS |= IF_RX_ACT;
            if ((USBPD->BMC_BYTE_CNT > 6u) &&
                ((PD_Rx_Buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC)) {
                goto chunk1_inline_ack;
            }
            if ((USBPD->BMC_BYTE_CNT == 6u) &&
                ((PD_Rx_Buf[0] & 0x1Fu) == DEF_TYPE_GOODCRC) && !got_crc) {
                PD_PHY.TxMsgID = (uint8_t)((PD_PHY.TxMsgID + 1u) & 0x07u);
                got_crc = 1;
                USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
                USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END | PD_DMA_EN;
                USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
                USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
                USBPD->CONTROL |= BMC_START;
            }
        }
        Delay_Us(3);
    }

    PD_Rx_Mode(); /* IRQ path still armed for late chunk1 */
    NVIC_EnableIRQ(USBPD_IRQn);
    if (!got_crc) {
        PD_PHY.TxMsgID = (uint8_t)((PD_PHY.TxMsgID + 1u) & 0x07u);
        printf("TX Chunk Request (GoodCRC miss) — still wait chunk 1\r\n");
    }
    return;

chunk1_inline_ack:
    pd_bytes_to_words(s_irq_rx_buf, PD_Rx_Buf, USBPD->BMC_BYTE_CNT);
    pd_defer_rx_packet(PD_PHY_RX_SOP);
    if (!got_crc) {
        PD_PHY.TxMsgID = (uint8_t)((PD_PHY.TxMsgID + 1u) & 0x07u);
    }
    Delay_Us(30);
    PD_Ack_Buf[0] = (uint8_t)(DEF_TYPE_GOODCRC | (PD_Rx_Buf[0] & 0xC0u));
    PD_Ack_Buf[1] = (PD_Rx_Buf[1] & 0x0Eu);
    /* mode=1: wait TX_END and re-arm RX (same as Chunk Request itself). */
    PD_Phy_SendPack(0x01, PD_Ack_Buf, 2, UPD_SOP0);
    NVIC_EnableIRQ(USBPD_IRQn);
}

void PD_TX_VCONN_DISC_IDENT(void)
{
    PD_TX_Reject();
}

void PD_TX_HRST(void)
{
    PD_Phy_SendPack(0x01, NULL, 0, UPD_HARD_RESET);
    PD_Rx_Mode();
}

static void PD_GPIO_CC_HiZ(void)
{
    GPIO_InitTypeDef gpio = {0};

    NVIC_DisableIRQ(USBPD_IRQn);
    USBPD->CONFIG &= ~(IE_RX_ACT | IE_RX_RESET);
    USBPD->CONTROL &= ~(PD_TX_EN | BMC_START);
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;
    gpio.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOB, &gpio);
}

static void PD_GPIO_USBPD_AF(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF4);
}

static void PD_Analyzer_PHY_Start(void)
{
    if (s_analyzer_phy_started) {
        return;
    }
    s_analyzer_phy_started = 1;
    PD_GPIO_USBPD_AF();
    AFIO->PCFR1 |= (1 << 20);
    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
    PD_SINK_Init();
    if (PD_Detect_Analyzer(0, 0) != 0) {
        s_analyzer_cable_at_boot = 1;
        s_analyzer_bounce_phase  = 1;
        s_analyzer_bounce_ms     = 0;
        PD_GPIO_CC_HiZ();
        printf("PD analyzer PHY start (cable present)\r\n");
    } else {
        PD_Rx_Mode();
        printf("PD analyzer PHY start\r\n");
    }
}

void PD_Analyzer_Init(void)
{
    PD_PHY_H417_Init();
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB | RCC_HB2Periph_AFIO, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBPD, ENABLE);
    PD_GPIO_CC_HiZ();
    s_analyzer_phy_started = 0;
    s_analyzer_boot_ms       = 0;
    s_analyzer_physical_cc   = 0;
    s_analyzer_connected     = 0;
    s_analyzer_active_cc     = 0;
    s_analyzer_bounce_phase    = 0;
    s_analyzer_bounce_ms       = 0;
    s_analyzer_cable_at_boot   = 0;
    s_analyzer_cc_tried_mask   = 0;
    s_analyzer_wake_bounce     = 0;
    s_analyzer_wake_ms         = 0;
    s_analyzer_wake_done       = 0;
    s_analyzer_wake_cycle      = 0;
    s_analyzer_softreset_cnt   = 0;
    s_analyzer_hrst_sent         = 0;
    s_analyzer_hrst_recovery     = 0;
    s_analyzer_hrst_reinit_done  = 0;
    s_analyzer_hrst_recovery_ms  = 0;
    PD_Analyzer_Reset_Attach_State();
    printf("PD init: analyzer, %s Rd\r\n",
           PD_SNK_USE_INTERNAL_RD ? "internal CC_PD controlled" : "external 5.1k");
}

void PD_Analyzer_Det_Proc(void)
{
    uint8_t status;

    if (!s_analyzer_phy_started) {
        if (s_analyzer_boot_ms < 1500u) {
            s_analyzer_boot_ms += Tmr_Ms_Dlt;
            return;
        }
        PD_Analyzer_PHY_Start();
        return;
    }

    if (s_analyzer_wake_bounce) {
        return;
    }

    if (s_analyzer_physical_cc) {
        /*
         * Sample only the active CC while connected.  The full two-line scan
         * rewrites both PORT_CC registers every 2ms and can corrupt BMC RX/TX
         * in progress, which intermittently dropped EPR chunks.
         */
        status = PD_Detect_Analyzer(1, s_analyzer_active_cc);
        if (status == 0) {
            if (!s_analyzer_hrst_recovery) {
                s_analyzer_det_cnt++;
                if (s_analyzer_det_cnt >= 10) {
                    s_analyzer_cc_tried_mask = 0;
                    PD_Analyzer_Do_Detach();
                }
            }
        } else if (status != s_analyzer_active_cc) {
            s_analyzer_det_cnt = 0;
            if (!(s_analyzer_cc_tried_mask & (uint8_t)(1u << (status - 1u)))) {
                printf("CC%d sees CC%d, switch\r\n", s_analyzer_active_cc, status);
                PD_Analyzer_Silent_CC_Swap(status);
            }
        } else {
            s_analyzer_det_cnt = 0;
        }
        return;
    }

    status = PD_Detect_Analyzer(0, 0);
    s_analyzer_det_cnt = 0;
}
