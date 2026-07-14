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

## Capturing results

The firmware emits versioned `@PD1` records alongside the detailed diagnostic
log. Use the Python capture tool instead of a generic serial terminal to split
hot-swapped sources into sessions and generate a stable summary:

```powershell
python -m pip install -r tools/requirements.txt
python tools/pd_capture.py --list
python tools/pd_capture.py --port COM3 --label aohi-240w
```

The default baud rate is 921600. The tool normally prints only completed or
updated summaries; add `--verbose` to echo every firmware debug line. Each run
is saved below `captures/` with the exact UART byte stream plus per-session
artifacts:

```text
captures/YYYYMMDD_HHMMSS_label/
  raw.bin
  raw.log
  events.jsonl
  session_0001/
    raw.log
    events.jsonl
    result.json
    summary.txt
```

Saved structured logs can be processed again without hardware using
`python tools/pd_capture.py --replay path/to/raw.log`.

Every capture stop also refreshes two aggregate capability tables:

- `captures/spec_table.html`: searchable source/cable/protocol table with links
  to each session's summary, JSON, and raw log
- `captures/spec_table.csv`: the same rows for Excel or further processing

Open `captures/spec_table.html` in a browser to compare adapters without
opening the individual session files. Existing captures can be re-indexed at
any time without connecting hardware:

```powershell
python tools/pd_report.py --captures captures
```

The table includes PD/USB revision, Source Capabilities Extended identity,
Source Info, manufacturer data, and a per-command support matrix when the
source responds to the optional information probes.

## Current behavior

- SPR fixed and PPS PDO decoding
- Optional PPS contract/status probe followed by restoration to fixed 5 V
- EPR Mode entry and chunked EPR Source Capabilities reconstruction
- Passive SOP' Discover Identity ACK decoding for E-marked cables
- Post-interrogation SOP Discover Identity, SVID, and Mode discovery
- Soft/Hard Reset recovery and Message ID tracking
- Cable detach/re-attach support without resetting the CH32H417
- Machine-readable source, cable, and protocol result records over UART
- Safe GET_* information probing with immediate fallback on Soft Reset,
  missing GoodCRC, or unsupported commands

Some sources may temporarily downgrade to SPR after a rejected request,
Soft Reset, cable discovery failure, or another policy-engine decision. The
firmware reports the capabilities actually advertised by the source and avoids
forcing repeated EPR entry attempts.

SOP' capture is deliberately passive and best-effort: the firmware never sends
VCONN Swap or SOP' requests that could interfere with source-to-cable traffic.

## Hardware note

This configuration expects external 5.1 kohm Rd resistors. Do not enable the
internal Rd at the same time unless the board hardware is changed accordingly.
