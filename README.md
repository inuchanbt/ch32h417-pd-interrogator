# CH32H417 PD Interrogator

USB Power Delivery sink and protocol interrogator firmware for the WCH
CH32H417. It negotiates a safe 5 V contract, enumerates SPR/PPS capabilities,
enters EPR mode when supported, reconstructs chunked EPR Source Capabilities,
and passively records SOP' cable identity traffic.

The firmware is based on the WCH CH32H417 EVT examples and currently targets
the nanoCH32H417 hardware with external 5.1 kohm CC pull-down resistors.

## Project layout

- `USBPD/USBPD_SNK/Common`: PD protocol, PHY, analyzer, and decoder logic
- `USBPD/USBPD_SNK/V3F`: primary V3F MounRiver project
- `USBPD/USBPD_SNK/V5F`: companion V5F MounRiver project
- `SRC`: WCH startup, peripheral, core, and linker support referenced by both
  projects

## Building

Open `USBPD/USBPD_SNK/USBPD_SNK.wvsln` with MounRiver Studio 2.

Build the V3F project first, then rebuild the V5F project. The image used for
programming is:

`USBPD/USBPD_SNK/V5F/obj/Merge.Bin`

The analyzer UART is configured for 921600 baud.

## Current behavior

- SPR fixed and PPS PDO decoding
- Optional PPS contract/status probe followed by restoration to fixed 5 V
- EPR Mode entry and chunked EPR Source Capabilities reconstruction
- Passive SOP' Discover Identity ACK decoding for E-marked cables
- Soft/Hard Reset recovery and Message ID tracking
- Cable detach/re-attach support without resetting the CH32H417

Some sources may temporarily downgrade to SPR after a rejected request,
Soft Reset, cable discovery failure, or another policy-engine decision. The
firmware reports the capabilities actually advertised by the source and avoids
forcing repeated EPR entry attempts.

## Hardware note

This configuration expects external 5.1 kohm Rd resistors. Do not enable the
internal Rd at the same time unless the board hardware is changed accordingly.

