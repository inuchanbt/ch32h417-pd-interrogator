/********************************** (C) COPYRIGHT *******************************
 * pd_msgtypes.h — PD message types (ported from ch32x035_usbpd.h for PD_Prot).
 ********************************************************************************/

#ifndef PD_MSGTYPES_H_
#define PD_MSGTYPES_H_

#include <stdint.h>

typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef volatile u8 vu8;

#define PD_PHY_HRST                 0xFFu

#define PD_Ctrl_Reserved            0x00
#define PD_Ctrl_GoodCRC             0x01
#define PD_Ctrl_GotoMin             0x02
#define PD_Ctrl_Accept              0x03
#define PD_Ctrl_Reject              0x04
#define PD_Ctrl_Ping                0x05
#define PD_Ctrl_PS_Ready            0x06
#define PD_Ctrl_GetSrcCap           0x07
#define PD_Ctrl_GetSinkCap          0x08
#define PD_Ctrl_DRSwap              0x09
#define PD_Ctrl_PRSwap              0x0A
#define PD_Ctrl_VconnSwap           0x0B
#define PD_Ctrl_Wait                0x0C
#define PD_Ctrl_SoftReset           0x0D
#define PD_Ctrl_DataReset           0x0E
#define PD_Ctrl_DataResetComplete   0x0F
#define PD_Ctrl_NotSupported        0x10
#define PD_Ctrl_GetSrcCapExtended   0x11
#define PD_Ctrl_GetStatus           0x12
#define PD_Ctrl_FRSwap              0x13
#define PD_Ctrl_GetPPSStatus        0x14
#define PD_Ctrl_GetCountryCodes     0x15
#define PD_Ctrl_GetSinkCapExtended  0x16
#define PD_Ctrl_GetSrcInfo          0x17
#define PD_Ctrl_GetRevision         0x18

#define PD_Data_Reserved1           0x00
#define PD_Data_SrcCap              0x01
#define PD_Data_Request             0x02
#define PD_Data_BIST                0x03
#define PD_Data_SinkCap             0x04
#define PD_Data_BatteryStatus       0x05
#define PD_Data_Alert               0x06
#define PD_Data_GetCountryInfo      0x07
#define PD_Data_EnterUSB            0x08
#define PD_Data_EPRRequest          0x09
#define PD_Data_EPRMode             0x0A
#define PD_Data_SrcInfo             0x0B
#define PD_Data_Revision            0x0C
#define PD_Data_Reserved2           0x0D
#define PD_Data_Reserved3           0x0E
#define PD_Data_VendorDefined       0x0F

#define PD_Ext_Reserved             0x00
#define PD_Ext_SrcCapExtended       0x01
#define PD_Ext_Status               0x02
#define PD_Ext_GetBatteryCap        0x03
#define PD_Ext_GetBatteryStatus     0x04
#define PD_Ext_BattertCap           0x05
#define PD_Ext_GetManufacturerInfo  0x06
#define PD_Ext_ManufacturerInfo     0x07
#define PD_Ext_SecurityRequest      0x08
#define PD_Ext_SecurityResponse     0x09
#define PD_Ext_FWUpdateRequest      0x0A
#define PD_Ext_FWUpdateResponse     0x0B
#define PD_Ext_PPSStatus            0x0C
#define PD_Ext_CountryInfo          0x0D
#define PD_Ext_CountryCodes         0x0E
#define PD_Ext_SinkCapExtended      0x0F
#define PD_Ext_EPRSrcCapabilities   0x11

#define PD_VDM_DiscoverIdentity     0x01
#define PD_VDM_DiscoverSVIDs        0x02
#define PD_VDM_DiscoverModes        0x03
#define PD_VDM_EnterModes           0x04
#define PD_VDM_ExitModes            0x05
#define PD_VDM_Attention            0x06

#define DEF_Unstructured_VDM        0
#define DEF_Structured_VDM          1

#define PD_Rev2                     0x01
#define PD_Rev3                     0x02

#define DevRole_Sink                0
#define DevRole_Src                 1
#define DevRole_DRP                 2
#define DevRole_DRP_TrySink         3
#define DevRole_DRP_TrySrc          4

#endif /* PD_MSGTYPES_H_ */
