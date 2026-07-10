/********************************** (C) COPYRIGHT  *******************************
* File Name          : hardware.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file contains all the functions prototypes for the 
*                      hardware.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __HARDWARE_H
#define __HARDWARE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32h417.h"
#include "debug.h"

/* Set to 1 when VBUS is routed to an ADC pin with a divider (not on nanoCH32H417). */
#ifndef PD_VBUS_USE_ADC
#define PD_VBUS_USE_ADC  0
#endif

void PD_VBUS_Init(void);
void PD_VBUS_Update(void);
UINT8 PD_VBUS_Valid(void);

void Hardware(void);

#ifdef __cplusplus
}
#endif

#endif 





