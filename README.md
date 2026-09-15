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

## Hardware preparation (nanoCH32H417 V1.0)

Before using the nanoCH32H417 V1.0 as a PD sink, enable its external
5.1 kΩ CC pull-down resistors by solder-bridging **both jumper locations
circled in red** in the photo below, next to the `USB-FS` USB-C connector.
Bridge the two pads within each circle separately; do not connect the two
circled locations to each other. Disconnect all power and USB cables before
soldering.

![nanoCH32H417 V1.0: the two CC resistor solder jumpers to bridge](docs/images/nanoch32h417-v1-cc-jumpers.png)

This is a required hardware setup step for this board, not a firmware setting.
The photo applies to the V1.0 board shown; check the jumper layout if using a
different board revision.

## Building

Open `USBPD/USBPD_SNK/USBPD_SNK.wvsln` with MounRiver Studio 2.

Build the V3F project first, then rebuild the V5F project. The image used for
programming is:

`USBPD/USBPD_SNK/V5F/obj/Merge.Bin`

The analyzer UART is configured for 460800 baud.

## Capturing results

The firmware emits versioned `@PD1` records alongside the detailed diagnostic
log. Use the Python capture tool instead of a generic serial terminal to split
hot-swapped sources into sessions and generate a stable summary:

```powershell
python -m pip install -r tools/requirements.txt
python tools/pd_capture.py --list
python tools/pd_capture.py --port COM3 --source-id aohi-240w --source-manufacturer AOHI --source-model AOC-C022 --cable-id cable-01 --cable-attachment detachable
```

The default baud rate is 460800. The tool echoes every firmware debug line by
default so attach and protocol progress remain visible; add `--quiet` to show
only completed or updated summaries. Source port defaults to `C1`. For a
`captive` cable, omitted cable manufacturer/model values inherit the source
manufacturer/model. Each run is saved below `captures/` with the exact UART
byte stream plus per-session artifacts:

```text
captures/YYYYMMDD_HHMMSS_SOURCE_MANUFACTURER_SOURCE_MODEL/
  capture.json
  SOURCE__CABLE__pd-interrogate__YYYYMMDD_HHMMSS.bin
  SOURCE__CABLE__pd-interrogate__YYYYMMDD_HHMMSS.log
  events.jsonl
  session_0001/
    SOURCE__CABLE__pd-interrogate__YYYYMMDD_HHMMSS__session-0001.log
    events.jsonl
    result.json
    summary.txt
```

Saved structured logs can be processed again without hardware using
`python tools/pd_capture.py --replay path/to/saved.log`. Old `raw.log` files
remain supported. `capture.json` and each `result.json` identify their log
filename under `artifacts.raw_log`; `capture.json` also records `artifacts.raw_binary`.

## Reusing measurement setups and searching history

Use `python tools/pd_capture.py --help` and `python tools/pd_report.py --help`
for all options and examples. Run the examples below from this project's root.

Start the interactive setup chooser with:

```powershell
python tools/pd_capture.py --setup
```

Choose a saved setup (the last used one is offered), or register a new one.
Saved setups include physical source/cable IDs, manufacturer/model, source port,
cable length, AC input, firmware version, board revision, CC resistors,
orientation, and a test note. Review the setup before starting capture.
No measurement starts when you only save a profile:

```powershell
python tools/pd_capture.py --save-profile bench-aohi --source-id aohi-240w --source-manufacturer AOHI --source-model AOC-C022 --cable-id cable-01 --firmware-version r70 --cc-resistance external-5.1k
python tools/pd_capture.py --profiles
python tools/pd_capture.py --profile bench-aohi --port COM3 --orientation ura
```

Explicit command-line options override saved values for that run without changing
the saved profile. `--save-profile NAME` updates that named profile. To register
metadata from an existing capture without retyping it:

```powershell
python tools/pd_capture.py --from-capture captures/EXISTING_CAPTURE_FOLDER --save-profile my-adapter
```

New log names follow ASD-PD31's `SOURCE__CABLE__CONDITION__YYYYMMDD_HHMMSS`
format and Windows filename sanitization, including `.` to `p`. Use `--source-name`
and `--cable-name` to specify the same readable labels as ASD-PD31; otherwise
labels are derived from device metadata. `--measurement-condition` defaults to
`pd-interrogate`. Long labels are shortened to fit the capture path; full values
remain in JSON. Capture directories are allocated exclusively; repeated runs in
the same second get `_02`, `_03`, etc., without overwriting earlier evidence.

`captures/measurements.sqlite3` stores saved profiles and a searchable session
index. Raw logs and per-session JSON remain the evidence; the index can be
refreshed from old and new capture directories without renaming or moving them:

```powershell
python tools/pd_report.py --history --source-id aohi-240w
python tools/pd_report.py --history --cable-id cable-01 --limit 100
```

Re-indexing preserves saved profiles, result IDs, and favorites. Back up the
whole `captures` directory, including the database (saved profiles) and
`result_annotations.json` (favorites); rebuilding the index does not recreate
profiles. Existing exports and dashboard matching columns remain supported.
CSV exports add measurement condition, firmware, board, CC configuration,
orientation, test note, capture ID, and relative raw-log path. Each CSV is replaced
only after writing completes so readers do not see partially written rows.

Every capture stop also refreshes the aggregate capability reports:

- `captures/spec_table.html`: searchable source/cable/protocol table with links
  to each session's summary, JSON, and raw log
- `captures/spec_table.csv`: the same rows for Excel or further processing
- `captures/source_table.csv`: one consolidated capability row per source ID
- `captures/pdo_table.csv`: one normalized row per advertised PDO, with raw
  values and source-role flags aligned with ASD-PD31 field names
- `captures/cable_table.csv`: one row per cable ID with decoded SOP' identity
  and the sources on which that cable was observed

`source_id` and `cable_id` identify independent devices. A session records the
temporary source/cable pairing, while PDO rows belong to `source_id` and cable
identity rows belong to `cable_id`. `--label` remains an alias for
`--source-id`. The human-readable capture folder suffix is built from
`--source-manufacturer` and `--source-model`; `source_id` remains the stable
CSV/database key. For older captures without an explicit cable ID, the report uses
an `auto-emarker-*` capability fingerprint derived from the captured VDOs; use
`--cable-id` when individual physical cables must remain distinguishable.

Use `--cable-attachment captive` for a source with a permanently attached
cable. The source and SOP' identity remain separate records, but
`source_table.csv` links the source to `captive_cable_id`, and
`cable_table.csv` links the cable back to `owner_source_id`. If `--cable-id`
is omitted for a captive cable, the capture tool derives
`source-id:captive`. Detachable observations remain temporary pairings and do
not create an ownership relationship.

Open `captures/spec_table.html` in a browser to compare adapters without
opening the individual session files. Existing captures can be re-indexed at
any time without connecting hardware:

```powershell
python tools/pd_report.py --captures captures
```

For the interactive Results Viewer with persistent favorites, start the local
server and open the printed URL (default: `http://127.0.0.1:8766/`):

```powershell
python tools/pd_report.py --captures captures --serve
```

The viewer classifies complete sessions as `Valid`, acquisition failures as
`Failed`, and partial sessions as `Review`. Favorites are saved outside the
immutable session evidence in `captures/result_annotations.json`. The generated
`spec_table.csv` includes `Result ID`, `Result Status`, and `Favorite` columns so
an ASD-PD31 import can preserve the annotation without automatically pairing the
two measurements.

The table includes PD/USB revision, Source Capabilities Extended identity,
Source Info, manufacturer data, and a per-command support matrix when the
source responds to the optional information probes.

## Current behavior

Host-tool checks (saved-log replay only; no measurement hardware required):

```powershell
python -m unittest discover -s tools -p "test*.py" -v
```

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
