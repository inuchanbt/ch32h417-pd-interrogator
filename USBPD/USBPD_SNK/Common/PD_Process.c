/********************************** (C) COPYRIGHT *******************************
* File Name          : PD_process.c
* Author             : WCH
* Version            : V1.0.
* Date               : 2026/04/08
* Description        : This file provides all the PD firmware functions.
*********************************************************************************
* Copyright (c) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "debug.h"
#include <string.h>
#include "PD_Process.h"
#include "pd_phy_h417.h"

extern void PD_VBUS_Update(void);
extern UINT8 PD_VBUS_Valid(void);

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Extended 7-object replies are 36 bytes on the wire: 2 header + 2 ext + 28 data + 4 CRC. */
__attribute__ ((aligned(4))) uint8_t PD_Rx_Buf[ 64 ];                           /* PD receive buffer */
__attribute__ ((aligned(4))) uint8_t PD_Tx_Buf[ 64 ];                           /* PD send buffer */

/******************************************************************************/
UINT8 PD_Ack_Buf[ 2 ];                                                          /* PD-ACK buffer */

UINT8  Tmr_Ms_Cnt_Last;                                                         /* System timer millisecond timing final value */
UINT8  Tmr_Ms_Dlt;                                                              /* System timer millisecond timing this interval value */

PD_CONTROL PD_Ctl;                                                              /* PD Control Related Structures */

UINT8  Adapter_SrcCap[ 30 ];                                                    /* SrcCap message from the adapter */

UINT8  PDO_Len;

/* Attach sequence: CC-only -> quiet attach wait -> RX/TX after VBUS or timeout */
#define PD_GETSRCCAP_TIMEOUT_MS  1000u
#define PD_ATTACH_DEBUG_MS       1000u
#define PD_GETSRCCAP_RETRY_CNT   20u
#define PD_GETSRCCAP_RETRY_MS    20u

/*
 * Bring-up test: leave PB3/PB4 disconnected from the USBPD PHY. Keep this at
 * 0 for normal PD negotiation.
 */
#ifndef PD_CC_HIZ_ONLY_TEST
#define PD_CC_HIZ_ONLY_TEST      0
#endif

/*
 * nanoCH32H417 default: bridge the semicircle jumpers at R5/R6 so external
 * 5.1k Rd is on CC1/CC2, then keep internal PHY pull-down off (0).
 *
 * Delta ADP-240KB BA cold-plug experiment:
 * CH32X035+CH211 can leave Rd physically absent during boot, then enable it
 * later through CH211. H417 can only mimic that if R5/R6 are open and CC relies
 * on internal CC_PD. Set this to 1 only in that hardware configuration.
 */
#ifndef PD_SNK_USE_INTERNAL_RD
#define PD_SNK_USE_INTERNAL_RD   0
#endif

#if PD_SNK_USE_INTERNAL_RD
#define PD_SNK_RD_CFG            CC_PD
#else
#define PD_SNK_RD_CFG            0
#endif

typedef enum
{
    PD_PHASE_CC_ONLY = 0,
    PD_PHASE_ATTACH_RX,
} PD_ATTACH_PHASE;

static PD_ATTACH_PHASE s_attach_phase = PD_PHASE_CC_ONLY;
static UINT16          s_attach_ms = 0;
static UINT8           s_active_cc = 0;
static UINT8           s_getsrc_cap_sent = 0;
static UINT8           s_got_unsolicited_srccap = 0;
static UINT16          s_fail_holdoff = 0;
static UINT16          s_attach_debug_ms = 0;
static UINT16          s_boot_quiet_ms = 0;
static UINT8           s_phy_started = 0;
static UINT8           s_last_cc1_cmp = 0;
static UINT8           s_last_cc2_cmp = 0;

static void PD_Attach_Reset( void );
static void PD_Attach_Quiet_Restart( void );
static void PD_Attach_Begin_Rx( UINT8 cc );
static void PD_Attach_Try_Get_Src_Cap( void );
static void PD_Rx_Stop( void );
static void PD_Select_CC( UINT8 cc );
static UINT8 PD_CC_Sample( __IO uint16_t *port );
static void PD_PHY_Start( void );
static void PD_GPIO_USBPD_AF( void );
static void PD_GPIO_CC_HiZ( void );

/* SrcCap Table */
UINT8 SrcCap_5V3A_Tab[ 4 ]  = { 0X2C, 0X91, 0X01, 0X3E };
UINT8 SrcCap_5V2A_Tab[ 4 ]  = { 0XC8, 0X90, 0X01, 0X3E };
UINT8 SinkCap_5V1A_Tab[ 4 ] = { 0X64, 0X90, 0X01, 0X36 };

/* PD3.0 */
UINT8 SrcCap_Ext_Tab[ 28 ] =
{
    0X18, 0X80, 0X63, 0X00,
    0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X01, 0X00,
    0X00, 0X00, 0X07, 0X03,
    0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X03,
    0X00, 0X12, 0X00, 0X00,
};

UINT8 Status_Ext_Tab[ 8 ] =
{
    0X06, 0X80, 0X16, 0X00,
    0X00, 0X00, 0X00, 0X00,
};

#if Func_Run_V3F

/*********************************************************************
 * @fn      USBPD_IRQHandler
 *
 * @brief   This function handles USBPD interrupt.
 *
 * @return  none
 */
void USBPD_IRQHandler(void)
{
#if PD_USE_ANALYZER
    PD_PHY_IRQHandler();
#else
    if(USBPD->STATUS & IF_RX_ACT)
    {
        USBPD->STATUS |= IF_RX_ACT;
        if( ( USBPD->STATUS & MASK_PD_STAT ) == PD_RX_SOP0 )
        {
            if( USBPD->BMC_BYTE_CNT >= 6 )
            {
                /* If GOODCRC, do not answer and ignore this reception */
                if( ( USBPD->BMC_BYTE_CNT != 6 ) || ( ( PD_Rx_Buf[ 0 ] & 0x1F ) != DEF_TYPE_GOODCRC ) )
                {
                    Delay_Us(30);                       /* Delay 30us, answer GoodCRC */
                    PD_Ack_Buf[ 0 ] = 0x41;
                    PD_Ack_Buf[ 1 ] = ( PD_Rx_Buf[ 1 ] & 0x0E ) | PD_Ctl.Flag.Bit.Auto_Ack_PRRole;
                    USBPD->CONFIG |= IE_TX_END ;
                    PD_Phy_SendPack( 0, PD_Ack_Buf, 2, UPD_SOP0 );
                }
            }
        }
    }
    if(USBPD->STATUS & IF_TX_END)
    {
        /* GoodCRC send completion after a received packet */
        USBPD->PORT_CC1 &= ~CC_LVE;
        USBPD->PORT_CC2 &= ~CC_LVE;

        /* Interrupts are turned off and can be turned on after the main function has finished processing the data */
        NVIC_DisableIRQ(USBPD_IRQn);

        PD_Ctl.Flag.Bit.Msg_Recvd = 1;
        USBPD->STATUS |= IF_TX_END;
    }
    if(USBPD->STATUS & IF_RX_RESET)
    {
        USBPD->STATUS |= IF_RX_RESET;
        PD_SINK_Init( );
        if( s_attach_phase != PD_PHASE_CC_ONLY )
        {
            PD_Rx_Mode( );
        }
        printf("IF_RX_RESET\r\n");
    }
#endif
}

#endif

/*********************************************************************
 * @fn      PD_Rx_Mode
 *
 * @brief   This function uses to enter reception mode.
 *
 * @return  none
 */
void PD_Rx_Mode( void )
{
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET|PD_DMA_EN;
    USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
    USBPD->CONTROL |= BMC_START ;
    NVIC_EnableIRQ( USBPD_IRQn );
}

/*********************************************************************
 * @fn      PD_Rx_Stop
 *
 * @brief   Stop BMC RX/TX; leave CC comparators under PD_SINK_Init.
 *
 * @return  none
 */
static void PD_Rx_Stop( void )
{
    NVIC_DisableIRQ( USBPD_IRQn );
    USBPD->CONFIG &= ~( IE_RX_ACT | IE_RX_RESET );
    USBPD->CONTROL &= ~( PD_TX_EN | BMC_START );
}

/*********************************************************************
 * @fn      PD_SRC_Init
 *
 * @brief   This function uses to initialize SRC mode.
 *
 * @return  none
 */
void PD_SRC_Init( )
{
    PD_Ctl.Flag.Bit.PR_Role = 1;                                          /* SRC mode */
    PD_Ctl.Flag.Bit.Auto_Ack_PRRole = 1;                                  /* Default auto-responder role is SRC */
    USBPD->PORT_CC1 = CC_CMP_66 | CC_PU_330;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PU_330;
}

/*********************************************************************
 * @fn      PD_SINK_Init
 *
 * @brief   This function uses to initialize SNK mode.
 *
 * @return  none
 */
void PD_SINK_Init( )
{
    PD_Ctl.Flag.Bit.PR_Role = 0;
    PD_Ctl.Flag.Bit.Auto_Ack_PRRole = 0;
    PD_Ctl.Flag.Bit.PD_Role = 0;
    USBPD->PORT_CC1 = CC_CMP_66 | PD_SNK_RD_CFG | CC_NO_PU;
    USBPD->PORT_CC2 = CC_CMP_66 | PD_SNK_RD_CFG | CC_NO_PU;
}

/*********************************************************************
 * @fn      PD_PHY_Reset
 *
 * @brief   This function uses to reset PD PHY.
 *
 * @return  none
 */
void PD_PHY_Reset( void )
{
    PD_Ctl.Flag.Bit.Msg_Recvd = 0;
    PD_Ctl.Msg_ID = 0;
    PD_Ctl.Flag.Bit.PD_Version = 0;
    PD_SINK_Init( );
    PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;
    PD_Ctl.PD_State = STA_IDLE;
    PD_Ctl.Flag.Bit.PD_Comm_Succ = 0;
}

/*********************************************************************
 * @fn      PD_Attach_Reset
 *
 * @brief   Return to CC-only sink attach wait (no BMC).
 *
 * @return  none
 */
static void PD_Attach_Reset( void )
{
    PD_Rx_Stop( );
    PD_PHY_Reset( );
    s_attach_phase = PD_PHASE_CC_ONLY;
    s_attach_ms = 0;
    s_active_cc = 0;
    s_getsrc_cap_sent = 0;
    s_got_unsolicited_srccap = 0;
    s_attach_debug_ms = 0;
    PD_Ctl.Det_Cnt = 0;
}

/*********************************************************************
 * @fn      PD_Attach_Quiet_Restart
 *
 * @brief   Back off to GPIO Hi-Z so the source sees only external Rd.
 *
 * @return  none
 */
static void PD_Attach_Quiet_Restart( void )
{
    PD_Rx_Stop( );
    PD_GPIO_CC_HiZ( );
    PD_Ctl.Flag.Bit.Msg_Recvd = 0;
    PD_Ctl.Msg_ID = 0;
    PD_Ctl.Flag.Bit.Connected = 0;
    PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;
    PD_Ctl.PD_State = STA_IDLE;
    PD_Ctl.Flag.Bit.PD_Comm_Succ = 0;
    s_attach_phase = PD_PHASE_CC_ONLY;
    s_attach_ms = 0;
    s_active_cc = 0;
    s_getsrc_cap_sent = 0;
    s_got_unsolicited_srccap = 0;
    s_attach_debug_ms = 0;
    s_boot_quiet_ms = 0;
    s_phy_started = 0;
    PD_Ctl.Det_Cnt = 0;
}

/*********************************************************************
 * @fn      PD_Attach_Begin_Rx
 *
 * @brief   Phase B: CC attach confirmed, enable PD RX.
 *
 * @return  none
 */
static void PD_Attach_Begin_Rx( UINT8 cc )
{
    s_active_cc = cc;
    s_attach_ms = 0;
    s_getsrc_cap_sent = 0;
    s_got_unsolicited_srccap = 0;
    PD_Select_CC( cc );
    PD_GPIO_CC_HiZ( );
    s_attach_phase = PD_PHASE_ATTACH_RX;
    printf("CC%d attach, quiet wait (L:%02X/%02X)\r\n", cc, s_last_cc1_cmp, s_last_cc2_cmp);
}

/*********************************************************************
 * @fn      PD_Init
 *
 * @brief   This function uses to initialize PD registers and states.
 *
 * @return  none
 */
void PD_Init( void )
{
    RCC_HB2PeriphClockCmd( RCC_HB2Periph_GPIOB|RCC_HB2Periph_AFIO, ENABLE);               /* Open PD I/O clock, AFIO clock and PD clock */
    RCC_HBPeriphClockCmd( RCC_HBPeriph_USBPD, ENABLE );
    PD_GPIO_CC_HiZ( );
#if PD_CC_HIZ_ONLY_TEST
    memset( &PD_Ctl.PD_State, 0x00, sizeof( PD_CONTROL ) );
    s_phy_started = 0;
    s_boot_quiet_ms = 0;
    s_attach_phase = PD_PHASE_CC_ONLY;
    printf("PD init: Hi-Z only test, external Rd only\r\n");
    return;
#endif
    AFIO->PCFR1|=(1<<20);
    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
    memset( &PD_Ctl.PD_State, 0x00, sizeof( PD_CONTROL ) );
    Adapter_SrcCap[ 0 ] = 1;
    memcpy( &Adapter_SrcCap[ 1 ], SrcCap_5V3A_Tab, 4 );
    s_phy_started = 0;
    s_boot_quiet_ms = 0;
    s_attach_phase = PD_PHASE_CC_ONLY;
    printf("PD init: Hi-Z CC, ext 5.1k Rd (int Rd %s)\r\n",
           PD_SNK_USE_INTERNAL_RD ? "on" : "off");
}

/*********************************************************************
 * @fn      PD_Detect
 *
 * @brief   This function uses to detect CC connection.
 *
 * @return  0:No connection; 1:CC1 connection; 2:CC2 connection
 */
UINT8 PD_Detect( void )
{
    UINT8  ret = 0;
    UINT8  cmp_cc1 = 0;
    UINT8  cmp_cc2 = 0;

    if(PD_Ctl.Flag.Bit.Connected)                                       /* Detect disconnection */
    {
        __IO uint16_t *port;

        if( s_active_cc == 2 )
        {
            port = &USBPD->PORT_CC2;
        }
        else
        {
            port = &USBPD->PORT_CC1;
        }

        *port &= ~( CC_CMP_Mask | PA_CC_AI );
        *port |= CC_CMP_22;
        Delay_Us( 2 );
        if( *port & PA_CC_AI )
        {
            ret = s_active_cc;
        }
        PD_SINK_Init( );
    }
    else                                                                /* Detect insertion */
    {
        cmp_cc1 = PD_CC_Sample( &USBPD->PORT_CC1 );
        cmp_cc2 = PD_CC_Sample( &USBPD->PORT_CC2 );
        s_last_cc1_cmp = cmp_cc1;
        s_last_cc2_cmp = cmp_cc2;

        /* One active CC only (Default Rp is ~0.4V, below CC_CMP_66). */
        if( cmp_cc1 && !cmp_cc2 )
        {
            ret = 1;
        }
        else if( cmp_cc2 && !cmp_cc1 )
        {
            ret = 2;
        }
        else if( cmp_cc1 && cmp_cc2 )
        {
            ret = 1;   /* Huawei A to C cable has two pull-up resistors */
        }

        /* Restore CC comparator for BMC; detect leaves CC at CC_CMP_22 otherwise */
        PD_SINK_Init( );
    }
    return( ret );
}

void PD_Analyzer_Notify_Attach( UINT8 cc )
{
    s_active_cc = cc;
    PD_Ctl.Flag.Bit.Connected = 1;
}

void PD_Analyzer_Notify_Detach( void )
{
    s_active_cc = 0;
    PD_Ctl.Flag.Bit.Connected = 0;
}

/*********************************************************************
 * @fn      PD_Detect_Analyzer
 *
 * @brief   CC detect for analyzer mode.  Avoids PD_SINK_Init every 4ms
 *          while connected (that rewrites PORT_CC during BMC and breaks
 *          deferred Request TX on Anker-class sources).
 *
 * @return  0: no connection; 1: CC1; 2: CC2
 */
UINT8 PD_Detect_Analyzer( UINT8 connected, UINT8 active_cc )
{
    UINT8  ret = 0;
    UINT8  cmp_cc1;
    UINT8  cmp_cc2;
    __IO uint16_t *port;

    if ( connected ) {
        if ( active_cc == 2 ) {
            port = &USBPD->PORT_CC2;
        } else {
            port = &USBPD->PORT_CC1;
        }
        *port &= ~( CC_CMP_Mask | PA_CC_AI );
        *port |= CC_CMP_22;
        Delay_Us( 2 );
        if ( *port & PA_CC_AI ) {
            ret = active_cc;
        }
        return ret;
    }

    cmp_cc1 = PD_CC_Sample( &USBPD->PORT_CC1 );
    cmp_cc2 = PD_CC_Sample( &USBPD->PORT_CC2 );
    s_last_cc1_cmp = cmp_cc1;
    s_last_cc2_cmp = cmp_cc2;
    if ( cmp_cc1 && !cmp_cc2 ) {
        ret = 1;
    } else if ( cmp_cc2 && !cmp_cc1 ) {
        ret = 2;
    } else if ( cmp_cc1 && cmp_cc2 ) {
        if ( cmp_cc2 > cmp_cc1 ) {
            ret = 2;
        } else if ( cmp_cc1 > cmp_cc2 ) {
            ret = 1;
        } else {
            ret = 2;
        }
    }
    if ( ret ) {
        PD_SINK_Init( );
    }
    return ret;
}

/*********************************************************************
 * @fn      PD_CC_Sample
 *
 * @brief   Sample a CC comparator at several thresholds for attach debug.
 *
 * @return  bit0: >0.22V, bit1: >0.66V, bit2: >1.23V
 */
static UINT8 PD_CC_Sample( __IO uint16_t *port )
{
    UINT8 cmp = 0;

    *port &= ~( CC_CMP_Mask | PA_CC_AI );
    *port |= CC_CMP_22;
    Delay_Us( 2 );
    if( *port & PA_CC_AI )
    {
        cmp |= 0x01;
    }

    *port &= ~( CC_CMP_Mask | PA_CC_AI );
    *port |= CC_CMP_66;
    Delay_Us( 2 );
    if( *port & PA_CC_AI )
    {
        cmp |= 0x02;
    }

    *port &= ~( CC_CMP_Mask | PA_CC_AI );
    *port |= CC_CMP_123;
    Delay_Us( 2 );
    if( *port & PA_CC_AI )
    {
        cmp |= 0x04;
    }

    return cmp;
}

/*********************************************************************
 * @fn      PD_PHY_Start
 *
 * @brief   Enable the USBPD PHY after the source has seen a quiet Rd.
 *
 * @return  none
 */
static void PD_PHY_Start( void )
{
    s_phy_started = 1;
    PD_GPIO_USBPD_AF( );
    PD_Attach_Reset( );
    printf("PD PHY start, wait attach\r\n");
}

/*********************************************************************
 * @fn      PD_GPIO_USBPD_AF
 *
 * @brief   Connect PB3/PB4 to the USBPD PHY.
 *
 * @return  none
 */
static void PD_GPIO_USBPD_AF( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource3, GPIO_AF4 );
    GPIO_PinAFConfig( GPIOB, GPIO_PinSource4, GPIO_AF4 );
}

/*********************************************************************
 * @fn      PD_GPIO_CC_HiZ
 *
 * @brief   Leave CC pins quiet so the source sees only the external Rd.
 *
 * @return  none
 */
static void PD_GPIO_CC_HiZ( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    PD_Rx_Stop( );
    USBPD->PORT_CC1 &= ~CC_LVE;
    USBPD->PORT_CC2 &= ~CC_LVE;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
}

/*********************************************************************
 * @fn      PD_Select_CC
 *
 * @brief   Select CC1 or CC2 for BMC communication.
 *
 * @return  none
 */
static void PD_Select_CC( UINT8 cc )
{
    if( cc == 1 )
    {
        USBPD->CONFIG &= ~CC_SEL;
    }
    else
    {
        USBPD->CONFIG |= CC_SEL;
    }
}

/*********************************************************************
 * @fn      PD_Try_Get_Src_Cap
 *
 * @brief   Send Get_Source_Cap on the selected CC line.
 *
 * @return  DEF_PD_TX_OK or DEF_PD_TX_FAIL
 */
static UINT8 PD_Try_Get_Src_Cap( void )
{
    PD_GPIO_USBPD_AF( );
    PD_SINK_Init( );
    PD_Select_CC( s_active_cc );
    PD_Rx_Mode( );
    PD_Load_Header( 0x00, DEF_TYPE_GET_SRC_CAP );
    return PD_Send_Handle( NULL, 0 );
}

/*********************************************************************
 * @fn      PD_Attach_Try_Get_Src_Cap
 *
 * @brief   Phase C: send Get_Source_Cap after VBUS valid or timeout.
 *
 * @return  none
 */
static void PD_Attach_Try_Get_Src_Cap( void )
{
    UINT8  tx_ok;
    UINT8  retry;

    if( s_getsrc_cap_sent || s_got_unsolicited_srccap || PD_Ctl.Flag.Bit.Connected )
    {
        return;
    }
    s_getsrc_cap_sent = 1;

    PD_Ctl.PD_State = STA_SRC_CONNECT;
    printf("CC%d SRC Connect\r\n", s_active_cc);
    PD_Rx_Mode( );
    Delay_Ms( 10 );
    tx_ok = PD_Try_Get_Src_Cap( );

    for( retry = 0; ( tx_ok != DEF_PD_TX_OK ) && ( retry < PD_GETSRCCAP_RETRY_CNT ); retry++ )
    {
        printf("Retry CC%d #%d\r\n", s_active_cc, retry + 1);
        PD_Select_CC( s_active_cc );
        Delay_Ms( PD_GETSRCCAP_RETRY_MS );
        tx_ok = PD_Try_Get_Src_Cap( );
    }
    if( tx_ok == DEF_PD_TX_OK )
    {
        PD_Ctl.Flag.Bit.Connected = 1;
        PD_Ctl.Flag.Bit.Stop_Det_Chk = 1;
        PD_Ctl.Err_Op_Cnt = 0;
        printf("Get_Src_Cap TX ok (CC%d)\r\n", s_active_cc);
        PD_Ctl.PD_Comm_Timer = 0;
    }
    else
    {
        printf("Get_Src_Cap TX fail ST=%08X\r\n", (unsigned)USBPD->STATUS);
        PD_Ctl.PD_State = STA_IDLE;
        PD_Attach_Quiet_Restart( );
    }
}

/*********************************************************************
 * @fn      PD_Det_Proc
 *
 * @brief   This function uses to process the return value of PD_Detect.
 *
 * @return  none
 */
void PD_Det_Proc( void )
{
    UINT8  status;
    static UINT16 boot_skip = 0;

#if PD_CC_HIZ_ONLY_TEST
    (void)status;
    return;
#endif

    if( boot_skip < 100 )
    {
        boot_skip++;
        return;
    }

    if( !s_phy_started )
    {
        s_boot_quiet_ms += Tmr_Ms_Dlt;
        if( s_boot_quiet_ms >= 1500 )
        {
            PD_PHY_Start( );
        }
        return;
    }

    if( s_fail_holdoff )
    {
        s_fail_holdoff--;
        return;
    }

    if( PD_Ctl.Flag.Bit.Connected )
    {
        if( PD_Ctl.Flag.Bit.Stop_Det_Chk )
        {
            return;
        }
        status = PD_Detect( );
        if( status == 0 )
        {
            PD_Ctl.Det_Cnt++;
        }
        else
        {
            PD_Ctl.Det_Cnt = 0;
        }
        if( PD_Ctl.Det_Cnt >= 10 )
        {
            printf("CC disconnect\r\n");
            PD_Ctl.Flag.Bit.Connected = 0;
            PD_Attach_Reset( );
            s_fail_holdoff = 200;
        }
    }
    else if( s_attach_phase == PD_PHASE_CC_ONLY )
    {
        status = PD_Detect( );
        if( status == 0 )
        {
            PD_Ctl.Det_Cnt = 0;
        }
        else
        {
            PD_Ctl.Det_Cnt++;
        }
        if( PD_Ctl.Det_Cnt >= 10 )
        {
            PD_Ctl.Det_Cnt = 0;
            if( PD_Ctl.Flag.Bit.Stop_Det_Chk == 0 )
            {
                PD_Attach_Begin_Rx( status );
            }
        }
    }
}

/*********************************************************************
 * @fn      PD_Phy_SendPack
 *
 * @brief   This function uses to send PD data.
 *
 * @return  none
 */
void PD_Phy_SendPack( UINT8 mode, UINT8 *pbuf, UINT8 len, UINT8 sop )
{
    UINT16 tx_wait;

    /* Always start from a clean RX/idle BMC state.
     * Stale IF_TX_END/BUF_ERR bits can make the next TX look complete without
     * putting a real BMC packet on CC. */
    USBPD->CONTROL &= ~( PD_TX_EN | BMC_START );
    USBPD->STATUS = 0xFF;

    if ((USBPD->CONFIG & CC_SEL) == CC_SEL )
    {
        USBPD->PORT_CC2 |= CC_LVE;
    }
    else
    {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = UPD_TMR_TX_120M;

    USBPD->USBPD_DMA = (UINT32)(UINT8 *)pbuf;

    USBPD->TX_SEL = sop;

    USBPD->BMC_TX_SZ = len;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS = 0xFF;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;

    /* Determine if you need to wait for the send to complete */
    if( mode )
    {
        /*
         * A missing TX_END must not freeze the analyzer forever.  The main
         * loop owns CC detach/re-attach detection, so always return to RX
         * within a bounded interval even after a PHY fault.
         */
        tx_wait = 3000;
        while( (USBPD->STATUS & IF_TX_END) == 0 && --tx_wait )
        {
            Delay_Us( 1 );
        }
        USBPD->CONTROL &= ~( PD_TX_EN | BMC_START );
        USBPD->STATUS = 0xFF;
        if((USBPD->CONFIG & CC_SEL) == CC_SEL )
        {
            USBPD->PORT_CC2 &= ~CC_LVE;
        }
        else
        {
            USBPD->PORT_CC1 &= ~CC_LVE;
        }

        /* Switch to receive ready to receive GoodCRC */
        USBPD->CONFIG |= PD_ALL_CLR;
        USBPD->CONFIG &= ~PD_ALL_CLR;
        USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | IE_TX_END | PD_DMA_EN;
        USBPD->USBPD_DMA = (UINT32)(UINT8 *)PD_Rx_Buf;
        USBPD->BMC_CLK_CNT = UPD_TMR_RX_120M;
        USBPD->CONTROL |= BMC_START;
    }
}

/*********************************************************************
 * @fn      PD_Load_Header
 *
 * @brief   This function uses to load pd header packets.
 *
 * @return  none
 */
void PD_Load_Header( UINT8 ex, UINT8 msg_type )
{
    /* Message Header
       BIT15 - Extended;
       BIT[14:12] - Number of Data Objects
       BIT[11:9] - Message ID
       BIT8 - PortPower Role/Cable Plug  0: SINK; 1: SOURCE
       BIT[7:6] - Revision, 00: V1.0; 01: V2.0; 10: V3.0;
       BIT5 - Port Data Role, 0: UFP; 1: DFP
       BIT[4:0] - Message Type
    */
    PD_Tx_Buf[ 0 ] = msg_type;
    if( PD_Ctl.Flag.Bit.PD_Role )
    {
        PD_Tx_Buf[ 0 ] |= 0x20;
    }
    if( PD_Ctl.Flag.Bit.PD_Version )
    {
        /* PD3.0 */
        PD_Tx_Buf[ 0 ] |= 0x80;
    }
    else
    {
        /* PD2.0 */
        PD_Tx_Buf[ 0 ] |= 0x40;
    }

    PD_Tx_Buf[ 1 ] = PD_Ctl.Msg_ID & 0x0E;
    if( PD_Ctl.Flag.Bit.PR_Role )
    {
        PD_Tx_Buf[ 1 ] |= 0x01;
    }
    if( ex )
    {
        PD_Tx_Buf[ 1 ] |= 0x80;
    }
}

/*********************************************************************
 * @fn      PD_Send_Handle
 *
 * @brief   This function uses to handle sending transactions.
 *
 * @return  0:success; 1:fail
 */
UINT8 PD_Send_Handle( UINT8 *pbuf, UINT8 len )
{
    UINT8  pd_tx_trycnt;
    UINT8  cnt;
    UINT16 goodcrc_wait;

    if( ( len % 4 ) != 0 )
    {
        /* Send failed */
        return( DEF_PD_TX_FAIL );
    }
    if( len > 28 )
    {
        /* Send failed */
        return( DEF_PD_TX_FAIL );
    }

    cnt = len >> 2;
    PD_Tx_Buf[ 1 ] |= ( cnt << 4 );
    for( cnt = 0; cnt != len; cnt++ )
    {
        PD_Tx_Buf[ 2 + cnt ] = pbuf[ cnt ];
    }

    pd_tx_trycnt = 4;
    while( --pd_tx_trycnt )                                                     /* Maximum 3 executions */
    {
        NVIC_DisableIRQ( USBPD_IRQn );
        PD_Phy_SendPack( 0x01, PD_Tx_Buf, ( len + 2 ), UPD_SOP0 );

        /* Set receive timeout ~1.5ms */
        goodcrc_wait = 500;
        while( --goodcrc_wait )
        {
            if( (USBPD->STATUS & IF_RX_ACT) == IF_RX_ACT)
            {
                USBPD->STATUS |= IF_RX_ACT;
                if( ( USBPD->BMC_BYTE_CNT == 6 ) && ( ( PD_Rx_Buf[ 0 ] & 0x1F ) == DEF_TYPE_GOODCRC ) )
                {
                    PD_Ctl.Msg_ID += 2;
                    break;
                }
            }
            Delay_Us( 3 );
        }
        if( goodcrc_wait != 0 )
        {
            break;
        }
    }

    /* Switch to receive mode */
    PD_Rx_Mode( );
    if( pd_tx_trycnt )
    {
        /* Send successful */
        return( DEF_PD_TX_OK );
    }
    else
    {
        /* Send failed */
        return( DEF_PD_TX_FAIL );
    }
}

/*********************************************************************
 * @fn      PDO_Request
 *
 * @brief   This function uses to Send the specified PDO.
 *
 * @return  none
 */
void PDO_Request( UINT8 pdo_index )
{
    UINT16 Current,Voltage;
    UINT8  status;
    if ((pdo_index > PDO_Len) || (pdo_index == 0))
    {
        while(1)
        {
            printf("pdo_index error!\r\n");
            Delay_Ms(500);
        }
    }
    else
    {
        memcpy( &PD_Rx_Buf[ 2 ], &Adapter_SrcCap[ 4*(pdo_index-1) + 1 ], 4 );
        PD_PDO_Analyse( 1, &PD_Rx_Buf[ 2 ], &Current, &Voltage );
        printf("Request:\r\nCurrent:%d mA\r\nVoltage:%d mV\r\n",Current,Voltage);

        PD_Load_Header( 0x00, DEF_TYPE_REQUEST );
        PD_Rx_Buf[ 5 ] = 0x03;
        PD_Rx_Buf[ 5 ] |= pdo_index<<4;
        PD_Rx_Buf[ 3 ] = PD_Rx_Buf[ 3 ] & 0x03;
        PD_Rx_Buf[ 3 ] |= ( PD_Rx_Buf[ 2 ] << 2 );
        PD_Rx_Buf[ 4 ] = PD_Rx_Buf[ 3 ];
        PD_Rx_Buf[ 4 ] <<= 2;
        PD_Rx_Buf[ 4 ] = PD_Rx_Buf[ 4 ] & 0x0C;
        PD_Rx_Buf[ 4 ] |= ( PD_Rx_Buf[ 2 ] >> 6 );
    }
    status = PD_Send_Handle( &PD_Rx_Buf[ 2 ], 4 );

    if( status == DEF_PD_TX_OK )
    {
        PD_Ctl.PD_State = STA_RX_ACCEPT_WAIT;
    }
    else
    {
        PD_Ctl.PD_State = STA_TX_SOFTRST;
    }
    PD_Ctl.PD_Comm_Timer = 0;
    PD_Ctl.Flag.Bit.PD_Comm_Succ = 1;
}

/*********************************************************************
 * @fn      PD_Save_Adapter_SrcCap
 *
 * @brief   This function uses to save the adapter SrcCap information.
 *
 * @return  none
 */
void PD_Save_Adapter_SrcCap( void )
{
    UINT8  i, len;

    /* Calculate the number of NDO's (Number of Data Objects) in the Message Header */
    len = ( ( PD_Rx_Buf[ 1 ] >> 4 ) & 0x07 );

    /* Remove the PPS section */
    for( i = 0; i < len; i++ )
    {
        if( ( PD_Rx_Buf[ 2 + ( i << 2 ) + 3 ] & 0xC0 ) == 0xC0 )
        {
            break;
        }
    }

    PDO_Len = i;

    /* Modify SrcCap information */
       /* BIT[31:30] - Fixed Supply */
       /* BIT29 - Dual-Role Power */
       /* BIT28 - USB Suspend Power */
       /* BIT27 - Unconstrained Power */
       /* BIT26 - USB Communications */
       /* BIT25 - Dual-Role Data */
       /* BIT24 - Unchunked Extended Message Supported */
       /* BIT23 - EPR Mode Capable */
       /* BIT22 - Reserved,shall be set to zero */
       /* BIT[21:20] - Peak Current */
       /* BIT[19:10] - Voltage in 50mV units */
       /* BIT[9:0] - Maximum Current in 10mA units */
    PD_Rx_Buf[ 5 ] = 0x3E;

    /* Save the adapter's SrcCap information */
    PD_Rx_Buf[ 1 ] &= 0x8F;
    PD_Rx_Buf[ 1 ] |= i << 4;
    Adapter_SrcCap[ 0 ] = i;
    memcpy( &Adapter_SrcCap[ 1 ], &PD_Rx_Buf[ 2 ], ( i << 2 ) );
}

/*********************************************************************
 * @fn      PD_PDO_Analyse
 *
 * @brief   This function uses to analyse PDO's voltage and current.
 *
 * @return  none
 */
void PD_PDO_Analyse( UINT8 pdo_idx, UINT8 *srccap, UINT16 *current, UINT16 *voltage )
{
    UINT32 temp32;

    temp32 = srccap[ (  ( pdo_idx - 1 ) << 2 ) + 0 ] +
                        ( (UINT32)srccap[ ( ( pdo_idx - 1 ) << 2 ) + 1 ] << 8 ) +
                        ( (UINT32)srccap[ ( ( pdo_idx - 1 ) << 2 ) + 2 ] << 16 );

    /* Calculation of current values */
    if( current != NULL )
    {
        *current = ( temp32 & 0x000003FF ) * 10;
    }

    /* Calculation of voltage values */
    if( voltage != NULL )
    {
        temp32 = temp32 >> 10;
        *voltage = ( temp32 & 0x000003FF ) * 50;
    }
}

/*********************************************************************
 * @fn      PD_Main_Proc
 *
 * @brief   This function uses to process PD status.
 *
 * @return  none
 */
void PD_Main_Proc( )
{
    UINT8  status;
    UINT8  pd_header;
    UINT8 var;
    UINT16 Current,Voltage;

    /* Receive idle timer count */
    PD_Ctl.PD_BusIdle_Timer += Tmr_Ms_Dlt;

    /* Phase B/C: after CC attach, listen first, then ask for Source_Cap. */
    if( ( !PD_Ctl.Flag.Bit.Connected ) && ( s_attach_phase == PD_PHASE_ATTACH_RX ) )
    {
        s_attach_ms += Tmr_Ms_Dlt;
        s_attach_debug_ms += Tmr_Ms_Dlt;
        PD_VBUS_Update( );
        if( !s_got_unsolicited_srccap && !s_getsrc_cap_sent )
        {
            if( PD_VBUS_Valid( ) || ( s_attach_ms >= PD_GETSRCCAP_TIMEOUT_MS ) )
            {
                if( PD_VBUS_Valid( ) )
                {
                    printf("VBUS valid, Get_Src_Cap\r\n");
                }
                else
                {
                    printf("Get_Src_Cap timeout %ums\r\n", (unsigned)s_attach_ms);
                }
                PD_Attach_Try_Get_Src_Cap( );
            }
            else if( s_attach_debug_ms >= PD_ATTACH_DEBUG_MS )
            {
                s_attach_debug_ms = 0;
                printf("CC%d wait VBUS %ums\r\n", s_active_cc, (unsigned)s_attach_ms);
            }
        }
    }

    /* Status analysis processing */
    switch( PD_Ctl.PD_State )
    {
        case STA_DISCONNECT:
            /* Status: Disconnected */
            printf("Disconnect\r\n");
            PD_Ctl.Flag.Bit.Connected = 0;
            PD_Attach_Reset( );
            break;

        case STA_SRC_CONNECT:
            /* Status: SRC access */
            /* If SRC_CAP is received within 1S, reset operation is performed */
            PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
            if( PD_Ctl.PD_Comm_Timer > 999 )
            {
                /* Retry on exception (abort after 5 attempts) */
                PD_Ctl.Err_Op_Cnt++;
                printf("SrcCap timeout (%d)\r\n", PD_Ctl.Err_Op_Cnt);
                if( PD_Ctl.Err_Op_Cnt > 5 )
                {
                    PD_Ctl.Err_Op_Cnt = 0;
                    PD_Ctl.Flag.Bit.Connected = 0;
                    PD_Ctl.PD_State = STA_IDLE;
                    PD_Attach_Reset( );
                }
                else
                {
                    PD_PHY_Reset( );
                    PD_Rx_Mode( );
                    PD_Load_Header( 0x00, DEF_TYPE_GET_SRC_CAP );
                    if( PD_Send_Handle( NULL, 0 ) != DEF_PD_TX_OK )
                    {
                        printf("Get_Src_Cap TX fail\r\n");
                    }
                }
                PD_Ctl.PD_Comm_Timer = 0;
            }
            break;

        case STA_RX_ACCEPT_WAIT:
            /* Status: waiting to receive ACCEPT */
        case STA_RX_PS_RDY_WAIT:
            /* Status: waiting to receive PS_RDY */
            PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
            if( PD_Ctl.PD_Comm_Timer > 499 )
            {
                PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;                         /* Enable connection detection*/
                PD_Ctl.PD_State = STA_TX_SOFTRST;
                PD_Ctl.PD_Comm_Timer = 0;
            }
            break;

        case STA_RX_PS_RDY:
            /* Status: PS_RDY received */
            PD_Ctl.PD_State = STA_IDLE;
            if( PD_Ctl.PD_State == STA_RX_APD_PS_RDY_WAIT )
            {
                PD_Ctl.PD_State = STA_RX_APD_PS_RDY;
            }
            break;

        case STA_TX_SOFTRST:
            /* Status: send software reset */
            /* Send soft reset, if sent successfully, mode unchanged, count +1 for retry */
            PD_Load_Header( 0x00, DEF_TYPE_SOFT_RESET );
            status = PD_Send_Handle( NULL, 0 );
            if( status == DEF_PD_TX_OK )
            {
                /* current mode unchanged, jump to initial state of current mode, mode retry count, switch mode if exceeded */
                PD_Ctl.PD_State = STA_IDLE;
            }
            else
            {
                PD_Ctl.PD_State = STA_TX_HRST;
            }
            PD_Ctl.PD_Comm_Timer = 0;
            break;

        case STA_TX_HRST:
            /* Status: Sending a hardware reset */
            /* Sending a hard reset */
            PD_Ctl.Flag.Bit.Stop_Det_Chk = 1;
            PD_Phy_SendPack( 0x01, NULL, 0, UPD_HARD_RESET );                   /* send HRST */
            PD_Rx_Mode( );                                                      /* switch to rx mode */
            PD_Ctl.PD_State = STA_IDLE;
            PD_Ctl.PD_Comm_Timer = 0;
            break;

        default:
            break;
    }

    /* Receive message processing */
    if( PD_Ctl.Flag.Bit.Msg_Recvd )
    {
        /* Adapter communication idle timing */
        PD_Ctl.Adapter_Idle_Cnt = 0x00;
        pd_header = PD_Rx_Buf[ 0 ] & 0x1F;
        printf("RX type:%02X\r\n", pd_header);
        switch( pd_header )
        {
            case DEF_TYPE_SRC_CAP:

                if( !PD_Ctl.Flag.Bit.Connected )
                {
                    s_got_unsolicited_srccap = 1;
                    PD_Ctl.Flag.Bit.Connected = 1;
                    PD_Ctl.Flag.Bit.Stop_Det_Chk = 1;
                    PD_Ctl.PD_State = STA_SRC_CONNECT;
                    PD_Ctl.PD_Comm_Timer = 0;
                    printf("unsolicited SrcCap (CC%d)\r\n", s_active_cc);
                }

                PD_Ctl.Err_Op_Cnt = 0;
                PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;

                PD_Save_Adapter_SrcCap( );
                PDO_Request( PDO_INDEX_1 );
                /* Analysis of the voltage and current of each PDO group */
                for (var = 1; var <= PDO_Len; ++var)
                {
                    PD_PDO_Analyse( var, &PD_Rx_Buf[ 2 ], &Current, &Voltage );
                    printf("PDO:%d\r\nCurrent:%d mA\r\nVoltage:%d mV\r\n",var,Current,Voltage);
                }
                printf("\r\n");
                /* Different PDO's for different voltages and currents */
                /* Default application for the first group of PDO, 5V */

                break;

            case DEF_TYPE_ACCEPT:
                /* ACCEPT received */
                PD_Ctl.PD_State = STA_RX_PS_RDY_WAIT;
                PD_Ctl.PD_Comm_Timer = 0;
                break;

            case DEF_TYPE_PS_RDY:
                /* PS_RDY is received */
                printf("Success\r\n");
                PD_Ctl.PD_State = STA_RX_PS_RDY;
                break;

            case DEF_TYPE_WAIT:
                /* WAIT received, many requests may receive WAIT, need specific analysis */
                break;

            case DEF_TYPE_GET_SNK_CAP:
                Delay_Ms( 1 );
                PD_Load_Header( 0x00, DEF_TYPE_SNK_CAP );
                PD_Send_Handle( SinkCap_5V1A_Tab, sizeof( SinkCap_5V1A_Tab ) );
                break;

            case DEF_TYPE_SOFT_RESET:
                Delay_Ms( 1 );
                PD_Load_Header( 0x00, DEF_TYPE_ACCEPT );
                PD_Send_Handle( NULL, 0 );
                break;

            case DEF_TYPE_GET_SRC_CAP_EX:
                Delay_Ms( 1 );
                PD_Load_Header( 0x01, DEF_TYPE_SRC_CAP );
                PD_Send_Handle( SrcCap_Ext_Tab, sizeof( SrcCap_Ext_Tab ) );
                break;

            case DEF_TYPE_GET_STATUS:
                Delay_Ms( 1 );
                PD_Load_Header( 0x01, DEF_TYPE_GET_STATUS_R );
                PD_Send_Handle( Status_Ext_Tab, sizeof( Status_Ext_Tab ) );
                break;

            case DEF_TYPE_VCONN_SWAP:
                Delay_Ms( 1 );
                PD_Load_Header( 0x00, DEF_TYPE_REJECT );
                PD_Send_Handle( NULL, 0 );
                break;

            case DEF_TYPE_VENDOR_DEFINED:
                /* VDM message handling */
                if( ( PD_Rx_Buf[ 2 ] & 0xC0 ) == 0 )
                {
                    /* REQ */
                    Delay_Ms( 1 );

                    /* Data to be sent is cached to PD_Tx_Buf */
                    PD_Load_Header( 0x00, DEF_TYPE_VENDOR_DEFINED );

                    /* Return to NAK */
                    if( ( PD_Rx_Buf[ 3 ] & 0x60 ) == 0 )
                    {
                        PD_Ctl.Flag.Bit.VDM_Version = 0;
                    }
                    else
                    {
                        PD_Ctl.Flag.Bit.VDM_Version = 1;
                    }
                    PD_Rx_Buf[ 2 ] |= 0x80;
                    PD_Send_Handle( &PD_Rx_Buf[ 2 ], 4 );
                }
                break;

            default:
            printf("Unsupported Command\r\n");
                break;
        }

        /* Message has been processed, interrupt reception is turned on again */
        PD_Rx_Mode( );
        PD_Ctl.Flag.Bit.Msg_Recvd = 0;                                    /* Clear the received flag */
        PD_Ctl.PD_BusIdle_Timer = 0;                                      /* Idle time cleared */
    }
}
