#!/usr/bin/env python3
"""Build an aggregate CH32H417 PD capability table from capture results."""

from __future__ import annotations

import argparse
import csv
import html
import json
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Iterable
from urllib.parse import quote


CAPTURE_RE = re.compile(
    r"^(?P<date>\d{8})_(?P<time>\d{6})(?:_(?P<label>.*))?$"
)
VALID_STATUS = {
    "unknown",
    "pending",
    "pass",
    "fail",
    "na",
    "unsupported",
    "no_response",
    "tx_failed",
    "nak",
    "busy",
    "rejected",
    "invalid",
}


def as_int(value: object, default: int = 0) -> int:
    try:
        return int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default


def format_power(mw: int) -> str:
    if mw <= 0:
        return "unknown"
    if mw % 1000 == 0:
        return f"{mw // 1000}W"
    return f"{mw / 1000:.1f}W"


def format_voltage(mv: int) -> str:
    if mv <= 0:
        return ""
    if mv % 1000 == 0:
        return f"{mv // 1000}V"
    return f"{mv / 1000:.1f}V"


def status_text(value: object) -> str:
    raw = str(value or "unknown")
    if raw not in VALID_STATUS:
        return "CORRUPT"
    return {"na": "N/A", "no_response": "NO RESPONSE", "tx_failed": "TX FAILED"}.get(
        raw, raw.upper()
    )


def exit_text(value: object) -> str:
    return {
        "ack": "ACK",
        "soft_reset": "Soft Reset",
        "source_cap": "Source Capabilities",
        "timeout": "Timeout",
        "unexpected": "Unexpected",
        "unknown": "N/A",
    }.get(str(value or "unknown"), "CORRUPT")


def capture_details(result_path: Path) -> tuple[str, str]:
    capture_name = result_path.parent.parent.name
    match = CAPTURE_RE.match(capture_name)
    if not match:
        return capture_name, capture_name
    stamp = datetime.strptime(
        match.group("date") + match.group("time"), "%Y%m%d%H%M%S"
    )
    label = match.group("label") or "unlabeled"
    return stamp.strftime("%Y-%m-%d %H:%M:%S"), label


def source_identity(identity: dict[str, object]) -> str:
    status = str(identity.get("status", "unknown"))
    if status != "pass":
        return status_text(status)
    try:
        vid = int(str(identity.get("vid", "0")), 16)
        pid = int(str(identity.get("pid", "0")), 16)
    except ValueError:
        return "CORRUPT"
    if vid == 0 and pid == 0:
        return "ACK, VID/PID unavailable"
    return f"ACK, VID 0x{vid:04X} / PID 0x{pid:04X}"


def cable_description(cable: dict[str, object]) -> tuple[str, str]:
    if cable.get("status") != "pass":
        return "Not captured", ""
    cable_type = {3: "Passive", 4: "Active"}.get(
        as_int(cable.get("cable_type")), "Unknown"
    )
    current_ma = as_int(cable.get("current_ma"))
    current = f"{current_ma // 1000}A" if current_ma else "default current"
    voltage = format_voltage(as_int(cable.get("max_mv"))) or "unknown voltage"
    speed = {
        0: "USB2 only",
        1: "USB3.2 Gen1",
        2: "USB4 Gen2",
        3: "USB4 Gen3",
        4: "USB4 Gen4",
    }.get(as_int(cable.get("usb_speed")), "Unknown")
    vid = str(cable.get("vid", "0000")).upper()
    return f"{cable_type} / {current} / {voltage} / {speed}", f"0x{vid}"


def epr_fixed_text(epr: dict[str, object]) -> str:
    fixed = epr.get("fixed", {})
    if not isinstance(fixed, dict):
        return ""
    voltages: list[str] = []
    for _, value in sorted(fixed.items(), key=lambda item: as_int(item[0])):
        if isinstance(value, list) and value:
            voltage = format_voltage(as_int(value[0]))
            if voltage:
                voltages.append(voltage)
    return ", ".join(voltages)


def probe_fields(data: dict[str, object]) -> tuple[str, str, str, str, str]:
    probes = data.get("probes", {})
    if not isinstance(probes, dict):
        return "Not recorded", "Not recorded", "Not recorded", "Not recorded", "Not recorded"
    statuses = probes.get("status", {})
    details = probes.get("details", {})
    if not isinstance(statuses, dict):
        statuses = {}
    if not isinstance(details, dict):
        details = {}

    revision = details.get("revision", {})
    if isinstance(revision, dict) and revision:
        revision_text = (
            f"PD {revision.get('pd_major', '?')}.{revision.get('pd_minor', '?')} / "
            f"USB {revision.get('usb_major', '?')}.{revision.get('usb_minor', '?')}"
        )
    else:
        revision_text = status_text(statuses.get("revision")) if "revision" in statuses else "Not recorded"

    ext = details.get("source_cap_ext", {})
    if isinstance(ext, dict) and ext:
        ext_text = (
            f"VID 0x{ext.get('vid', '0000')} / PID 0x{ext.get('pid', '0000')} / "
            f"XID 0x{ext.get('xid', '00000000')}"
        )
    else:
        ext_text = status_text(statuses.get("source_cap_ext")) if "source_cap_ext" in statuses else "Not recorded"

    info = details.get("source_info", {})
    if isinstance(info, dict) and info:
        input_name = {"0": "None", "1": "AC", "2": "DC", "3": "AC+DC"}.get(
            str(info.get("present_input", "")), "Unknown"
        )
        info_text = (
            f"PDP {info.get('pdp_w', '?')}W / Temp raw {info.get('temp_raw', '?')} / "
            f"Input {input_name}"
        )
    else:
        info_text = status_text(statuses.get("source_info")) if "source_info" in statuses else "Not recorded"

    manufacturer = details.get("manufacturer", {})
    if isinstance(manufacturer, dict) and manufacturer:
        name = str(manufacturer.get("name", "unavailable")).replace("_", " ")
        manufacturer_text = (
            f"{name} / VID 0x{manufacturer.get('vid', '0000')} / "
            f"PID 0x{manufacturer.get('pid', '0000')}"
        )
    else:
        manufacturer_text = (
            status_text(statuses.get("manufacturer_info"))
            if "manufacturer_info" in statuses
            else "Not recorded"
        )

    labels = {
        "revision": "Revision",
        "source_cap_ext": "SourceCapExt",
        "status": "Status",
        "source_info": "SourceInfo",
        "pps_status": "PPSStatus",
        "country_codes": "CountryCodes",
        "manufacturer_info": "Manufacturer",
        "battery_cap": "BatteryCap",
        "battery_status": "BatteryStatus",
    }
    support = "; ".join(
        f"{labels.get(str(name), str(name))}={status_text(value)}"
        for name, value in statuses.items()
    ) or "Not recorded"
    return revision_text, ext_text, info_text, manufacturer_text, support


def make_row(result_path: Path, captures: Path) -> dict[str, str]:
    data = json.loads(result_path.read_text(encoding="utf-8"))
    source = data.get("source", {})
    source_summary = source.get("summary", {}) if isinstance(source, dict) else {}
    epr = data.get("epr", {})
    epr_summary = epr.get("summary", {}) if isinstance(epr, dict) else {}
    identity = data.get("identity", {})
    cable = data.get("cable", {})
    protocol = data.get("protocol", {})
    captured, label = capture_details(result_path)

    pdo_count = as_int(source_summary.get("pdo_count"))
    pps_count = as_int(source_summary.get("pps_count"))
    max_power = format_power(as_int(source_summary.get("max_mw")))
    epr_capable = as_int(source_summary.get("epr_capable")) == 1
    epr_complete = as_int(epr_summary.get("complete")) == 1
    fixed = epr_fixed_text(epr if isinstance(epr, dict) else {})
    avs_min = format_voltage(as_int(epr_summary.get("avs_min_mv")))
    avs_max = format_voltage(as_int(epr_summary.get("avs_max_mv")))
    pdp = as_int(epr_summary.get("pdp_w"))
    cable_text, cable_vid = cable_description(cable if isinstance(cable, dict) else {})
    revision, extended_identity, source_info, manufacturer, probe_support = probe_fields(data)

    if not epr_capable:
        epr_fixed = "Not advertised"
        avs = "N/A"
    elif not epr_complete:
        epr_fixed = "Data unavailable"
        avs = "N/A"
    else:
        epr_fixed = fixed or "None"
        avs = f"{avs_min}-{avs_max}" if avs_min and avs_max else "None"

    issues: list[str] = []
    overall = str(data.get("overall", "PARTIAL")).upper()
    if overall != "PASS":
        issues.append("Measurement incomplete")
    for key in ("spr", "pps", "epr", "chunk"):
        if status_text(protocol.get(key)) == "CORRUPT":
            issues.append("Corrupt UART result record")
            break
    if epr_capable and not epr_complete:
        issues.append("EPR data unavailable")
    if cable_text == "Not captured":
        issues.append("Cable identity not captured")
    if not bool(data.get("detached")):
        issues.append("Detach not recorded")

    relative_session = result_path.parent.relative_to(captures)
    link_base = quote(relative_session.as_posix(), safe="/")
    return {
        "Captured": captured,
        "Label": label,
        "Session": str(data.get("session", "")),
        "Firmware session": str(data.get("firmware_session", "")),
        "Overall": overall,
        "SPR": f"{pdo_count} PDO / Max {max_power}",
        "PPS": "None" if pps_count == 0 else f"{pps_count} APDO",
        "EPR fixed": epr_fixed,
        "AVS": avs,
        "PDP": f"{pdp}W" if pdp else "N/A",
        "Source identity": source_identity(identity if isinstance(identity, dict) else {}),
        "PD revision": revision,
        "Extended identity": extended_identity,
        "Source info": source_info,
        "Manufacturer": manufacturer,
        "Probe support": probe_support,
        "Cable": cable_text,
        "Cable VID": cable_vid,
        "SPR contract": status_text(protocol.get("spr")),
        "PPS probe": status_text(protocol.get("pps")),
        "EPR enter": status_text(protocol.get("epr")),
        "Chunking": status_text(protocol.get("chunk")),
        "EPR exit": exit_text(protocol.get("exit")),
        "Detached": "Yes" if bool(data.get("detached")) else "No",
        "Notes": "; ".join(dict.fromkeys(issues)) or "None",
        "_summary": f"{link_base}/summary.txt",
        "_result": f"{link_base}/result.json",
        "_raw": f"{link_base}/raw.log",
    }


CSV_FIELDS = [
    "Captured",
    "Label",
    "Session",
    "Firmware session",
    "Overall",
    "SPR",
    "PPS",
    "EPR fixed",
    "AVS",
    "PDP",
    "Source identity",
    "PD revision",
    "Extended identity",
    "Source info",
    "Manufacturer",
    "Probe support",
    "Cable",
    "Cable VID",
    "SPR contract",
    "PPS probe",
    "EPR enter",
    "Chunking",
    "EPR exit",
    "Detached",
    "Notes",
]


def discover_rows(captures: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for result_path in captures.glob("*/session_*/result.json"):
        try:
            rows.append(make_row(result_path, captures))
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            print(f"Skipping {result_path}: {exc}", file=sys.stderr)
    rows.sort(key=lambda row: (row["Captured"], as_int(row["Session"])), reverse=True)
    return rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def html_cell(value: str, class_name: str = "") -> str:
    class_attr = f' class="{class_name}"' if class_name else ""
    return f"<td{class_attr}>{html.escape(value)}</td>"


def write_html(path: Path, rows: list[dict[str, str]]) -> None:
    body: list[str] = []
    for row in rows:
        overall = row["Overall"]
        status_class = "pass" if overall == "PASS" else "fail" if overall == "FAIL" else "partial"
        links = (
            f'<a href="{row["_summary"]}">Summary</a> '
            f'<a href="{row["_result"]}">JSON</a> '
            f'<a href="{row["_raw"]}">Raw</a>'
        )
        search = html.escape(" ".join(row[field] for field in CSV_FIELDS).lower(), quote=True)
        cells = [
            html_cell(row["Captured"]),
            html_cell(row["Label"], "label"),
            html_cell(row["Overall"], f"status {status_class}"),
            html_cell(row["SPR"]),
            html_cell(row["PPS"]),
            html_cell(row["EPR fixed"]),
            html_cell(row["AVS"]),
            html_cell(row["PDP"]),
            html_cell(row["Source identity"]),
            html_cell(row["PD revision"]),
            html_cell(row["Extended identity"]),
            html_cell(row["Source info"]),
            html_cell(row["Manufacturer"]),
            html_cell(row["Probe support"], "notes"),
            html_cell(row["Cable"]),
            html_cell(row["Cable VID"]),
            html_cell(row["SPR contract"]),
            html_cell(row["PPS probe"]),
            html_cell(row["EPR enter"]),
            html_cell(row["Chunking"]),
            html_cell(row["EPR exit"]),
            html_cell(row["Detached"]),
            html_cell(row["Notes"], "notes"),
            f'<td class="files">{links}</td>',
        ]
        body.append(
            f'<tr data-overall="{html.escape(overall)}" data-search="{search}">'
            + "".join(cells)
            + "</tr>"
        )

    generated = datetime.now().astimezone().isoformat(timespec="seconds")
    document = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CH32H417 PD Spec Table</title>
<style>
:root {{ color-scheme: light dark; font-family: Segoe UI, Arial, sans-serif; }}
* {{ box-sizing: border-box; }}
body {{ margin: 0; background: #f4f5f7; color: #20242a; }}
header {{ padding: 18px 24px 12px; border-bottom: 1px solid #c9ced6; background: #fff; }}
h1 {{ margin: 0 0 5px; font-size: 22px; letter-spacing: 0; }}
.meta {{ color: #5d6672; font-size: 13px; }}
.controls {{ display: flex; gap: 10px; align-items: center; padding: 12px 24px; background: #e9edf2; border-bottom: 1px solid #c9ced6; }}
input, select {{ min-height: 34px; padding: 6px 9px; border: 1px solid #aeb6c1; border-radius: 4px; background: #fff; color: #20242a; }}
input {{ width: min(460px, 55vw); }}
#count {{ margin-left: auto; color: #5d6672; font-size: 13px; }}
.table-wrap {{ overflow: auto; height: calc(100vh - 137px); }}
table {{ width: max-content; min-width: 100%; border-collapse: separate; border-spacing: 0; background: #fff; font-size: 13px; }}
th {{ position: sticky; top: 0; z-index: 2; padding: 8px 9px; text-align: left; white-space: nowrap; background: #343b45; color: #fff; border-right: 1px solid #59616d; }}
td {{ max-width: 270px; padding: 7px 9px; border-right: 1px solid #d9dde3; border-bottom: 1px solid #d9dde3; vertical-align: top; white-space: nowrap; }}
tbody tr:nth-child(even) {{ background: #f7f8fa; }}
tbody tr:hover {{ background: #e8f1fb; }}
.label {{ font-weight: 650; }}
.status {{ font-weight: 750; }}
.status.pass {{ color: #08783e; }}
.status.partial {{ color: #986300; }}
.status.fail {{ color: #b42318; }}
.notes {{ white-space: normal; min-width: 190px; }}
.files a {{ margin-right: 8px; color: #0969b5; }}
@media (prefers-color-scheme: dark) {{
  body, table {{ background: #17191d; color: #e6e9ed; }}
  header {{ background: #20242a; border-color: #444b55; }}
  .controls {{ background: #282d34; border-color: #444b55; }}
  input, select {{ background: #17191d; color: #e6e9ed; border-color: #59616d; }}
  th {{ background: #30363e; }}
  td {{ border-color: #3b414a; }}
  tbody tr:nth-child(even) {{ background: #1d2025; }}
  tbody tr:hover {{ background: #25384a; }}
  .status.pass {{ color: #56d38b; }}
  .status.partial {{ color: #f0bd54; }}
  .status.fail {{ color: #ff8177; }}
  .files a {{ color: #75bfff; }}
}}
</style>
</head>
<body>
<header>
  <h1>CH32H417 PD Spec Table</h1>
  <div class="meta">Generated {html.escape(generated)} from {len(rows)} captured session(s)</div>
</header>
<div class="controls">
  <input id="search" type="search" placeholder="Filter by label, PDO, VID, cable, status..." aria-label="Filter measurements">
  <select id="overall" aria-label="Overall result">
    <option value="">All results</option>
    <option value="PASS">PASS</option>
    <option value="PARTIAL">PARTIAL</option>
    <option value="FAIL">FAIL</option>
  </select>
  <span id="count"></span>
</div>
<div class="table-wrap">
<table>
<thead><tr>
  <th>Captured</th><th>Label</th><th>Overall</th><th>SPR</th><th>PPS</th>
  <th>EPR Fixed</th><th>AVS</th><th>PDP</th><th>Source Identity</th>
  <th>PD Revision</th><th>Extended Identity</th><th>Source Info</th>
  <th>Manufacturer</th><th>Probe Support</th><th>Cable</th><th>Cable VID</th>
  <th>SPR Contract</th><th>PPS Probe</th>
  <th>EPR Enter</th><th>Chunking</th><th>EPR Exit</th><th>Detached</th>
  <th>Notes</th><th>Files</th>
</tr></thead>
<tbody>{''.join(body)}</tbody>
</table>
</div>
<script>
const search = document.getElementById('search');
const overall = document.getElementById('overall');
const rows = Array.from(document.querySelectorAll('tbody tr'));
const count = document.getElementById('count');
function filterRows() {{
  const query = search.value.trim().toLowerCase();
  let visible = 0;
  for (const row of rows) {{
    const show = (!query || row.dataset.search.includes(query)) &&
      (!overall.value || row.dataset.overall === overall.value);
    row.hidden = !show;
    if (show) visible++;
  }}
  count.textContent = `${{visible}} / ${{rows.length}} sessions`;
}}
search.addEventListener('input', filterRows);
overall.addEventListener('change', filterRows);
filterRows();
</script>
</body>
</html>
"""
    path.write_text(document, encoding="utf-8")


def generate_reports(captures: Path) -> tuple[Path, Path, int]:
    captures.mkdir(parents=True, exist_ok=True)
    rows = discover_rows(captures)
    csv_path = captures / "spec_table.csv"
    html_path = captures / "spec_table.html"
    write_csv(csv_path, rows)
    write_html(html_path, rows)
    return html_path, csv_path, len(rows)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", type=Path, default=Path("captures"))
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    html_path, csv_path, count = generate_reports(args.captures.resolve())
    print(f"Wrote {html_path}")
    print(f"Wrote {csv_path}")
    print(f"Sessions: {count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
