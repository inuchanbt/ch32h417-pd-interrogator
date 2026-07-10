/********************************** (C) COPYRIGHT *******************************
 * ch211_stub.h — nanoCH32H417 has no CH211; CC/Rd via USBPD PHY directly.
 ********************************************************************************/

#ifndef CH211_STUB_H_
#define CH211_STUB_H_

#include "pd_msgtypes.h"

#define PD_Disable_Discharge
#define PD_Enable_Discharge
#define PD_Enable_Rd()
#define PD_Disable_Rd()
#define PD_Vconn_CC1
#define PD_Vconn_CC2
#define PD_Float_CC1
#define PD_Float_CC2

/* CH211 register aliases (stubbed on H417) */
#define PIN_STAT  0x10u
#define CCI1      0x01u
#define CCI2      0x02u

u8 CH211_I2C_ReadByte(u8 addr);

#endif /* CH211_STUB_H_ */
