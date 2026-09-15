#!/usr/bin/env python3
"""Build an aggregate CH32H417 PD capability table from capture results."""

from __future__ import annotations

import argparse
import csv
import io
import sqlite3
import hashlib
import html
import json
import os
import re
import socket
import sys
import threading
import uuid
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Iterable
from urllib.parse import quote, unquote, urlparse
from pd_store import index_rows, history, DATABASE_NAME


DEFAULT_CAPTURES = Path(__file__).resolve().parent.parent / "captures"
ANNOTATIONS_FILE = "result_annotations.json"
ANNOTATIONS_LOCK = threading.Lock()
RESULT_ID_RE = re.compile(r"^ch32h417:[A-Za-z0-9._-]{8,96}$")


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


def atomic_write_text(path: Path, text: str) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(text, encoding="utf-8", newline="")
    try:
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def annotations_path(captures: Path) -> Path:
    return captures / ANNOTATIONS_FILE


def load_annotations(captures: Path) -> dict[str, object]:
    path = annotations_path(captures)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {"schema_version": 1, "results": {}}
    except (OSError, json.JSONDecodeError, TypeError):
        return {"schema_version": 1, "results": {}}
    if not isinstance(data, dict) or not isinstance(data.get("results"), dict):
        return {"schema_version": 1, "results": {}}
    return data


def save_favorite(captures: Path, result_id: str, favorite: bool) -> dict[str, object]:
    if not RESULT_ID_RE.fullmatch(result_id):
        raise ValueError("invalid result_id")
    with ANNOTATIONS_LOCK:
        data = load_annotations(captures)
        results = data.setdefault("results", {})
        if not isinstance(results, dict):
            results = {}
            data["results"] = results
        if favorite:
            current = results.get(result_id, {})
            entry = dict(current) if isinstance(current, dict) else {}
            entry.update(
                {
                    "favorite": True,
                    "updated_at": datetime.now(timezone.utc).astimezone().isoformat(
                        timespec="seconds"
                    ),
                }
            )
            results[result_id] = entry
        else:
            results.pop(result_id, None)
        data["schema_version"] = 1
        atomic_write_text(
            annotations_path(captures),
            json.dumps(data, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        )
        return data


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


def format_current(ma: int) -> str:
    if ma % 1000 == 0:
        return f"{ma // 1000}A"
    return f"{ma / 1000:.2f}".rstrip("0").rstrip(".") + "A"


def format_voltage_range(min_mv: int, max_mv: int) -> str:
    return f"{format_voltage(min_mv).removesuffix('V')}-{format_voltage(max_mv)}"


def spr_avs_capability_text(decoded: dict[str, object]) -> str:
    min_mv = as_int(decoded.get("min_mv"))
    max_mv = as_int(decoded.get("max_mv"))
    ma_15v = as_int(decoded.get("max_ma_15v"))
    ma_20v = as_int(decoded.get("max_ma_20v"))
    ranges = [f"9-15V @ {format_current(ma_15v)}"]
    if ma_20v:
        ranges.append(f"15-20V @ {format_current(ma_20v)}")
    return f"{format_voltage_range(min_mv, max_mv)} ({' / '.join(ranges)})"


def parse_pdo(value: object) -> int:
    try:
        return int(str(value), 16) & 0xFFFFFFFF
    except (TypeError, ValueError):
        return 0


def decode_pdo(raw: int, object_number: int, region: str) -> dict[str, object]:
    pdo_type = (raw >> 30) & 0x03
    decoded: dict[str, object] = {
        "region": region,
        "object_number": object_number,
        "raw": f"0x{raw:08X}",
        "kind": "UNKNOWN",
        "min_mv": 0,
        "max_mv": 0,
        "max_ma": 0,
        "max_ma_15v": 0,
        "max_ma_20v": 0,
        "max_mw": 0,
        "pdp_w": 0,
        "power_limited": "",
        "peak_current": "",
    }
    if pdo_type == 0:
        mv = ((raw >> 10) & 0x3FF) * 50
        ma = (raw & 0x3FF) * 10
        decoded.update(
            kind=f"FIXED_{region}", min_mv=mv, max_mv=mv, max_ma=ma,
            max_mw=mv * ma // 1000,
        )
        description = f"Fixed {format_voltage(mv)} @ {format_current(ma)}"
    elif pdo_type == 1:
        max_mv = ((raw >> 20) & 0x3FF) * 50
        min_mv = ((raw >> 10) & 0x3FF) * 50
        max_mw = (raw & 0x3FF) * 250
        decoded.update(
            kind=f"BATTERY_{region}", min_mv=min_mv, max_mv=max_mv,
            max_mw=max_mw,
        )
        description = (
            f"Battery {format_voltage(min_mv)}-{format_voltage(max_mv)} "
            f"{format_power(max_mw)}"
        )
    elif pdo_type == 2:
        max_mv = ((raw >> 20) & 0x3FF) * 50
        min_mv = ((raw >> 10) & 0x3FF) * 50
        ma = (raw & 0x3FF) * 10
        decoded.update(
            kind=f"VARIABLE_{region}", min_mv=min_mv, max_mv=max_mv,
            max_ma=ma, max_mw=max_mv * ma // 1000,
        )
        description = (
            f"Variable {format_voltage_range(min_mv, max_mv)} "
            f"@ {format_current(ma)}"
        )
    else:
        apdo_type = (raw >> 28) & 0x03
        if apdo_type == 0:
            max_mv = ((raw >> 17) & 0xFF) * 100
            min_mv = ((raw >> 8) & 0xFF) * 100
            ma = (raw & 0x7F) * 50
            limited = bool((raw >> 27) & 1)
            decoded.update(
                kind="APDO_SPR_PPS", min_mv=min_mv, max_mv=max_mv,
                max_ma=ma, max_mw=max_mv * ma // 1000,
                power_limited=str(limited),
            )
            description = (
                f"PPS {format_voltage_range(min_mv, max_mv)} "
                f"@ {format_current(ma)}"
            )
        elif apdo_type == 1:
            max_mv = ((raw >> 17) & 0x1FF) * 100
            min_mv = ((raw >> 8) & 0x1FF) * 100
            pdp_w = raw & 0xFF
            decoded.update(
                kind="APDO_EPR_AVS", min_mv=min_mv, max_mv=max_mv,
                max_mw=pdp_w * 1000, pdp_w=pdp_w,
            )
            description = (
                f"AVS {format_voltage_range(min_mv, max_mv)} "
                f"PDP {pdp_w}W"
            )
        elif apdo_type == 2:
            ma_15v = ((raw >> 10) & 0x3FF) * 10
            ma_20v = (raw & 0x3FF) * 10
            max_mv = 20000 if ma_20v else 15000
            peak = (raw >> 26) & 0x03
            decoded.update(
                kind="APDO_SPR_AVS", min_mv=9000, max_mv=max_mv,
                max_ma=max(ma_15v, ma_20v), max_ma_15v=ma_15v,
                max_ma_20v=ma_20v,
                max_mw=max(15000 * ma_15v, 20000 * ma_20v) // 1000,
                peak_current=str(peak),
            )
            description = f"SPR AVS {spr_avs_capability_text(decoded)} Peak {peak}"
        else:
            description = "Reserved APDO"
    decoded["description"] = f"#{object_number} {description} ({decoded['raw']})"
    return decoded


def source_capability_fields(source: dict[str, object]) -> tuple[str, str, str, str]:
    pdos = source.get("pdos", {})
    if not isinstance(pdos, dict):
        return "Not recorded", "Not recorded", "Not recorded", "Not recorded"
    raw_pdos = {
        as_int(number): parse_pdo(raw)
        for number, raw in pdos.items()
        if parse_pdo(raw)
    }
    first = raw_pdos.get(1, 0)
    if not first:
        return "Not recorded", "Not recorded", "Not recorded", "Not recorded"
    roles = (
        f"Power: {'DRP' if (first >> 29) & 1 else 'Source'} / "
        f"Data: {'DRD' if (first >> 25) & 1 else 'DFP'}"
    )
    flags = [
        name for bit, name in (
            (28, "USB Suspend"),
            (27, "Unconstrained"),
            (26, "USB Communications"),
            (24, "Unchunked Extended"),
            (23, "EPR capable"),
        ) if (first >> bit) & 1
    ]
    flags.append(f"Peak current {(first >> 20) & 0x03}")
    details = "; ".join(
        str(decode_pdo(raw, number, "SPR")["description"])
        for number, raw in sorted(raw_pdos.items())
    )
    raw_text = "; ".join(
        f"#{number}=0x{raw:08X}" for number, raw in sorted(raw_pdos.items())
    )
    return roles, ", ".join(flags), details, raw_text


def epr_pdo_fields(epr: dict[str, object]) -> tuple[str, str]:
    pdos = epr.get("pdos", {})
    if not isinstance(pdos, dict):
        return "Not recorded", "Not recorded"
    raw_pdos = {
        as_int(number): parse_pdo(raw)
        for number, raw in pdos.items()
        if as_int(number) >= 8 and parse_pdo(raw)
    }
    if not raw_pdos:
        return "None", "None"
    details = "; ".join(
        str(decode_pdo(raw, number, "EPR")["description"])
        for number, raw in sorted(raw_pdos.items())
    )
    raw_text = "; ".join(
        f"#{number}=0x{raw:08X}" for number, raw in sorted(raw_pdos.items())
    )
    return details, raw_text


def spr_avs_fields(source: dict[str, object]) -> str:
    pdos = source.get("pdos", {})
    if not isinstance(pdos, dict):
        return "None"
    values: list[str] = []
    for number_text, raw_text in sorted(pdos.items(), key=lambda item: as_int(item[0])):
        number = as_int(number_text)
        raw = parse_pdo(raw_text)
        decoded = decode_pdo(raw, number, "SPR")
        if decoded["kind"] == "APDO_SPR_AVS":
            values.append(spr_avs_capability_text(decoded))
    return "; ".join(values) if values else "None"


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


def capture_metadata(result_path: Path) -> dict[str, object]:
    path = result_path.parent.parent / "capture.json"
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, TypeError, json.JSONDecodeError):
        return {}


def has_nonzero_hex(value: object) -> bool:
    try:
        return int(str(value), 16) != 0
    except (TypeError, ValueError):
        return False


def effective_cable(data: dict[str, object]) -> dict[str, object]:
    cable = data.get("cable", {})
    if isinstance(cable, dict) and (
        cable.get("status") == "pass" or has_nonzero_hex(cable.get("cable_vdo"))
    ):
        return cable
    identity = data.get("identity", {})
    if isinstance(identity, dict) and has_nonzero_hex(identity.get("cable_vdo")):
        return identity
    return cable if isinstance(cable, dict) else {}


def automatic_cable_id(cable: dict[str, object]) -> str:
    if cable.get("status") != "pass" and not has_nonzero_hex(cable.get("cable_vdo")):
        return ""
    values = [
        str(cable.get(name, default)).upper().removeprefix("0X")
        for name, default in (
            ("vid", "0000"),
            ("pid", "0000"),
            ("id_header", "00000000"),
            ("product_vdo", "00000000"),
            ("cable_vdo", "00000000"),
        )
    ]
    capability = "-".join(
        str(as_int(cable.get(name)))
        for name in ("cable_type", "current_ma", "max_mv", "usb_speed")
    )
    return "auto-emarker-" + "-".join(values) + "-" + capability


def capture_ids(
    result_path: Path,
    data: dict[str, object],
    cable: dict[str, object],
) -> tuple[str, str]:
    _, folder_source_id = capture_details(result_path)
    metadata = capture_metadata(result_path)
    source_id = str(data.get("source_id") or metadata.get("source_id") or folder_source_id)
    cable_id = str(
        data.get("cable_id")
        or metadata.get("cable_id")
        or automatic_cable_id(cable)
    )
    return source_id, cable_id


def cable_attachment(
    result_path: Path,
    data: dict[str, object],
    cable_id: str,
) -> str:
    metadata = capture_metadata(result_path)
    value = str(
        data.get("cable_attachment")
        or metadata.get("cable_attachment")
        or "unknown"
    ).lower()
    if value in ("detachable", "captive"):
        return value
    if cable_id.startswith("auto-emarker-"):
        return "detachable"
    return "unknown"


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
        return "Not observed this session", ""
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
        ext_parts = [
            f"VID 0x{ext.get('vid', '0000')} / PID 0x{ext.get('pid', '0000')} / "
            f"XID 0x{ext.get('xid', '00000000')}"
        ]
        if "fw_version" in ext or "hw_version" in ext:
            ext_parts.append(
                f"FW {ext.get('fw_version', '?')} / HW {ext.get('hw_version', '?')}"
            )
        if "spr_pdp_w" in ext:
            pdp = f"SPR PDP {ext.get('spr_pdp_w', '?')}W"
            if "epr_pdp_w" in ext:
                pdp += f" / EPR PDP {ext.get('epr_pdp_w', '?')}W"
            ext_parts.append(pdp)
        ext_text = " / ".join(ext_parts)
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


def probe_detail_text(data: dict[str, object]) -> str:
    probes = data.get("probes", {})
    details = probes.get("details", {}) if isinstance(probes, dict) else {}
    if not isinstance(details, dict) or not details:
        return "Not recorded"
    groups: list[str] = []
    metadata = {"format", "session", "type"}
    for name, values in details.items():
        if not isinstance(values, dict):
            continue
        fields = [
            f"{key}={value}"
            for key, value in values.items()
            if key not in metadata
        ]
        if fields:
            groups.append(f"{name}: " + ", ".join(fields))
    return "; ".join(groups) or "Not recorded"


def identity_raw_text(identity: dict[str, object]) -> str:
    fields = (
        ("ID Header", "id_header"),
        ("Product VDO", "product_vdo"),
    )
    values = [
        f"{label}=0x{str(identity.get(key, '00000000')).upper()}"
        for label, key in fields
        if key in identity
    ]
    return "; ".join(values) or "Not recorded"


def cable_raw_text(cable: dict[str, object]) -> str:
    fields = (
        ("ID Header", "id_header"),
        ("Product VDO", "product_vdo"),
        ("Cable VDO", "cable_vdo"),
    )
    values = [
        f"{label}=0x{str(cable.get(key, '00000000')).upper()}"
        for label, key in fields
        if key in cable
    ]
    pid = str(cable.get("pid", "")).upper()
    if pid:
        values.append(f"PID=0x{pid}")
    return "; ".join(values) or "Not recorded"


def result_id_for(result_path: Path, data: dict[str, object]) -> str:
    explicit = str(data.get("result_id") or "").strip()
    if RESULT_ID_RE.fullmatch(explicit):
        return explicit
    captured, _ = capture_details(result_path)
    identity = {
        "captured": captured,
        "capture": result_path.parent.parent.name,
        "session": data.get("session"),
        "firmware_session": data.get("firmware_session"),
        "source_id": data.get("source_id"),
        "result": data,
    }
    encoded = json.dumps(
        identity, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return "ch32h417:" + hashlib.sha256(encoded).hexdigest()[:32]


def result_status(overall: str) -> tuple[str, str]:
    if overall == "PASS":
        return "valid", "Valid"
    if overall == "FAIL":
        return "failed", "Failed"
    return "review", "Review"


def make_row(
    result_path: Path,
    captures: Path,
    annotations: dict[str, object] | None = None,
) -> dict[str, str]:
    data = json.loads(result_path.read_text(encoding="utf-8"))
    source = data.get("source", {})
    source_summary = source.get("summary", {}) if isinstance(source, dict) else {}
    epr = data.get("epr", {})
    epr_summary = epr.get("summary", {}) if isinstance(epr, dict) else {}
    identity = data.get("identity", {})
    cable = effective_cable(data)
    protocol = data.get("protocol", {})
    captured, _ = capture_details(result_path)
    metadata = capture_metadata(result_path)
    source_id, cable_id = capture_ids(result_path, data, cable)
    attachment = cable_attachment(result_path, data, cable_id)

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
    probe_details = probe_detail_text(data)
    source_roles, source_flags, spr_pdos, spr_pdo_raw = source_capability_fields(
        source if isinstance(source, dict) else {}
    )
    spr_avs = spr_avs_fields(source if isinstance(source, dict) else {})
    epr_pdos, epr_pdo_raw = epr_pdo_fields(epr if isinstance(epr, dict) else {})

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
    status_key, status_label = result_status(overall)
    result_id = result_id_for(result_path, data)
    annotation_results = (annotations or {}).get("results", {})
    annotation = (
        annotation_results.get(result_id, {})
        if isinstance(annotation_results, dict)
        else {}
    )
    favorite = bool(annotation.get("favorite")) if isinstance(annotation, dict) else False
    if overall != "PASS":
        issues.append("Measurement incomplete")
    for key in ("spr", "pps", "epr", "chunk"):
        if status_text(protocol.get(key)) == "CORRUPT":
            issues.append("Corrupt UART result record")
            break
    if epr_capable and not epr_complete:
        issues.append("EPR data unavailable")
    if not bool(data.get("detached")):
        issues.append("Detach not recorded")

    relative_session = result_path.parent.relative_to(captures)
    link_base = quote(relative_session.as_posix(), safe="/")
    artifacts = data.get("artifacts", {})
    raw_name = str(artifacts.get("raw_log") or "raw.log") if isinstance(artifacts, dict) else "raw.log"
    if Path(raw_name).name != raw_name or "/" in raw_name or "\\" in raw_name:
        raw_name = "raw.log"
    return {
        "Result ID": result_id,
        "Result Status": status_label,
        "Favorite": "Yes" if favorite else "No",
        "Captured": captured,
        "Source Manufacturer": str(metadata.get("source_manufacturer") or ""),
        "Source Model": str(metadata.get("source_model") or ""),
        "Source Port": str(metadata.get("source_port") or ""),
        "Cable Manufacturer": str(metadata.get("cable_manufacturer") or ""),
        "Cable Model": str(metadata.get("cable_model") or ""),
        "Cable Length m": str(metadata.get("cable_length_m") or ""),
        "Source ID": source_id,
        "Cable ID": cable_id,
        "Cable Attachment": attachment.capitalize(),
        "Session": str(data.get("session", "")),
        "Firmware session": str(data.get("firmware_session", "")),
        "Overall": overall,
        "SPR": f"{pdo_count} PDO / Max {max_power}",
        "PPS": "None" if pps_count == 0 else f"{pps_count} APDO",
        "SPR AVS": spr_avs,
        "EPR fixed": epr_fixed,
        "AVS": avs,
        "PDP": f"{pdp}W" if pdp else "N/A",
        "Power/data roles": source_roles,
        "Source flags": source_flags,
        "SPR PDO details": spr_pdos,
        "SPR PDO raw": spr_pdo_raw,
        "EPR PDO details": epr_pdos,
        "EPR PDO raw": epr_pdo_raw,
        "Source identity": source_identity(identity if isinstance(identity, dict) else {}),
        "Source identity raw": identity_raw_text(identity if isinstance(identity, dict) else {}),
        "PD revision": revision,
        "Extended identity": extended_identity,
        "Source info": source_info,
        "Manufacturer": manufacturer,
        "Probe support": probe_support,
        "Probe details": probe_details,
        "Cable": cable_text,
        "Cable observation": (
            "Observed passively via SOP\u2032"
            if cable_text != "Not observed this session"
            else "Not observed (not a capability failure)"
        ),
        "Cable VID": cable_vid,
        "Cable raw": cable_raw_text(cable if isinstance(cable, dict) else {}),
        "SPR contract": status_text(protocol.get("spr")),
        "PPS probe": status_text(protocol.get("pps")),
        "EPR enter": status_text(protocol.get("epr")),
        "Chunking": status_text(protocol.get("chunk")),
        "EPR exit": exit_text(protocol.get("exit")),
        "Detached": "Yes" if bool(data.get("detached")) else "No",
        "Notes": "; ".join(dict.fromkeys(issues)) or "None",
        "Measurement Condition": str(metadata.get("measurement_condition") or ""),
        "Firmware Version": str(metadata.get("firmware_version") or ""),
        "Board Revision": str(metadata.get("board_revision") or ""),
        "CC Resistance": str(metadata.get("cc_resistance") or ""),
        "Orientation": str(metadata.get("orientation") or ""),
        "Test Note": str(metadata.get("test_note") or ""),
        "Capture ID": str(metadata.get("capture_id") or ""),
        "Raw Log": f"{relative_session.as_posix()}/{raw_name}",
        "_summary": f"{link_base}/summary.txt",
        "_result": f"{link_base}/result.json",
        "_raw": f"{link_base}/{quote(raw_name, safe='')}",
    }


CSV_FIELDS = [
    "Result ID",
    "Result Status",
    "Favorite",
    "Captured",
    "Source Manufacturer",
    "Source Model",
    "Source Port",
    "Cable Manufacturer",
    "Cable Model",
    "Cable Length m",
    "Source ID",
    "Cable ID",
    "Cable Attachment",
    "Session",
    "Firmware session",
    "Overall",
    "SPR",
    "PPS",
    "SPR AVS",
    "EPR fixed",
    "AVS",
    "PDP",
    "Power/data roles",
    "Source flags",
    "SPR PDO details",
    "SPR PDO raw",
    "EPR PDO details",
    "EPR PDO raw",
    "Source identity",
    "Source identity raw",
    "PD revision",
    "Extended identity",
    "Source info",
    "Manufacturer",
    "Probe support",
    "Probe details",
    "Cable",
    "Cable observation",
    "Cable VID",
    "Cable raw",
    "SPR contract",
    "PPS probe",
    "EPR enter",
    "Chunking",
    "EPR exit",
    "Detached",
    "Notes",
    "Measurement Condition", "Firmware Version", "Board Revision", "CC Resistance",
    "Orientation", "Test Note", "Capture ID", "Raw Log",
]


def discover_rows(captures: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    annotations = load_annotations(captures)
    for result_path in captures.glob("*/session_*/result.json"):
        try:
            rows.append(make_row(result_path, captures, annotations))
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            print(f"Skipping {result_path}: {exc}", file=sys.stderr)
    rows.sort(key=lambda row: (row["Captured"], as_int(row["Session"])), reverse=True)
    return rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with io.StringIO(newline="") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        atomic_write_text(path, "\ufeff" + output.getvalue())


SOURCE_CSV_FIELDS = [
    "source_id",
    "captive_cable_id",
    "observation_count",
    "first_captured",
    "last_captured",
    "observed_cable_ids",
    "best_overall",
    "spr_summary",
    "pps_summary",
    "spr_avs",
    "epr_fixed",
    "avs",
    "pdp",
    "power_data_roles",
    "source_flags",
    "spr_pdo_details",
    "spr_pdo_raw",
    "epr_pdo_details",
    "epr_pdo_raw",
    "source_identity",
    "source_identity_raw",
    "pd_revision",
    "extended_identity",
    "source_info",
    "manufacturer",
    "probe_support",
    "probe_details",
]


def source_row_score(row: dict[str, str]) -> tuple[int, int, int]:
    overall = {"PASS": 2, "PARTIAL": 1}.get(row["Overall"], 0)
    epr = 1 if row["EPR PDO raw"] not in ("None", "Not recorded") else 0
    detail_fields = (
        "Source identity", "PD revision", "Extended identity", "Source info",
        "Manufacturer", "Probe details",
    )
    details = sum(
        row[field] not in ("", "N/A", "None", "NO RESPONSE", "Not recorded")
        for field in detail_fields
    )
    return overall, epr, details


def discover_source_rows(session_rows: list[dict[str, str]]) -> list[dict[str, object]]:
    groups: dict[str, dict[str, object]] = {}
    for session in session_rows:
        source_id = session["Source ID"]
        group = groups.get(source_id)
        if group is None:
            group = {
                "source_id": source_id,
                "observation_count": 0,
                "first_captured": session["Captured"],
                "last_captured": session["Captured"],
                "_cable_ids": set(),
                "_captive_cable_ids": set(),
                "_best": session,
            }
            groups[source_id] = group
        group["observation_count"] = as_int(group["observation_count"]) + 1
        group["first_captured"] = min(
            str(group["first_captured"]), session["Captured"]
        )
        group["last_captured"] = max(
            str(group["last_captured"]), session["Captured"]
        )
        cable_ids = group["_cable_ids"]
        if session["Cable ID"] and isinstance(cable_ids, set):
            cable_ids.add(session["Cable ID"])
        captive_cable_ids = group["_captive_cable_ids"]
        if (
            session["Cable Attachment"] == "Captive"
            and session["Cable ID"]
            and isinstance(captive_cable_ids, set)
        ):
            captive_cable_ids.add(session["Cable ID"])
        best = group["_best"]
        if isinstance(best, dict) and source_row_score(session) > source_row_score(best):
            group["_best"] = session

    result: list[dict[str, object]] = []
    mapping = {
        "best_overall": "Overall",
        "spr_summary": "SPR",
        "pps_summary": "PPS",
        "spr_avs": "SPR AVS",
        "epr_fixed": "EPR fixed",
        "avs": "AVS",
        "pdp": "PDP",
        "power_data_roles": "Power/data roles",
        "source_flags": "Source flags",
        "spr_pdo_details": "SPR PDO details",
        "spr_pdo_raw": "SPR PDO raw",
        "epr_pdo_details": "EPR PDO details",
        "epr_pdo_raw": "EPR PDO raw",
        "source_identity": "Source identity",
        "source_identity_raw": "Source identity raw",
        "pd_revision": "PD revision",
        "extended_identity": "Extended identity",
        "source_info": "Source info",
        "manufacturer": "Manufacturer",
        "probe_support": "Probe support",
        "probe_details": "Probe details",
    }
    for group in groups.values():
        best = group.pop("_best")
        cable_ids = group.pop("_cable_ids")
        captive_cable_ids = group.pop("_captive_cable_ids")
        group["observed_cable_ids"] = (
            ";".join(sorted(cable_ids)) if isinstance(cable_ids, set) else ""
        )
        group["captive_cable_id"] = (
            ";".join(sorted(captive_cable_ids))
            if isinstance(captive_cable_ids, set) else ""
        )
        if isinstance(best, dict):
            group.update({target: best[source] for target, source in mapping.items()})
        result.append(group)
    result.sort(key=lambda row: str(row["source_id"]))
    return result


def write_source_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with io.StringIO(newline="") as output:
        writer = csv.DictWriter(output, fieldnames=SOURCE_CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        atomic_write_text(path, "\ufeff" + output.getvalue())


PDO_CSV_FIELDS = [
    "source_id",
    "pdo_region",
    "pdo_object_number",
    "pdo_raw_u32",
    "pdo_kind",
    "min_voltage_mv",
    "max_voltage_mv",
    "max_current_ma",
    "max_current_15v_ma",
    "max_current_20v_ma",
    "max_power_mw",
    "pdp_w",
    "power_limited",
    "peak_current",
    "dual_role_power",
    "usb_suspend_supported",
    "unconstrained_power",
    "usb_communications_capable",
    "dual_role_data",
    "unchunked_extended_supported",
    "epr_capable",
    "observation_count",
    "first_captured",
    "last_captured",
    "observed_cable_ids",
    "description",
]


def discover_pdo_rows(captures: Path) -> list[dict[str, object]]:
    observations: list[dict[str, object]] = []
    for result_path in captures.glob("*/session_*/result.json"):
        try:
            data = json.loads(result_path.read_text(encoding="utf-8"))
            captured, _ = capture_details(result_path)
            cable = effective_cable(data)
            source_id, cable_id = capture_ids(result_path, data, cable)
            source = data.get("source", {})
            epr = data.get("epr", {})
            source_pdos = source.get("pdos", {}) if isinstance(source, dict) else {}
            epr_pdos = epr.get("pdos", {}) if isinstance(epr, dict) else {}
            first = parse_pdo(source_pdos.get("1")) if isinstance(source_pdos, dict) else 0
            common = {
                "captured": captured,
                "source_id": source_id,
                "cable_id": cable_id,
                "dual_role_power": bool((first >> 29) & 1),
                "usb_suspend_supported": bool((first >> 28) & 1),
                "unconstrained_power": bool((first >> 27) & 1),
                "usb_communications_capable": bool((first >> 26) & 1),
                "dual_role_data": bool((first >> 25) & 1),
                "unchunked_extended_supported": bool((first >> 24) & 1),
                "epr_capable": bool((first >> 23) & 1),
            }
            groups = (("SPR", source_pdos, 1), ("EPR", epr_pdos, 8))
            for region, pdos, first_object in groups:
                if not isinstance(pdos, dict):
                    continue
                for number_text, raw_text in pdos.items():
                    number = as_int(number_text)
                    raw = parse_pdo(raw_text)
                    if number < first_object or raw == 0:
                        continue
                    decoded = decode_pdo(raw, number, region)
                    observations.append({
                        **common,
                        "pdo_region": decoded["region"],
                        "pdo_object_number": decoded["object_number"],
                        "pdo_raw_u32": decoded["raw"],
                        "pdo_kind": decoded["kind"],
                        "min_voltage_mv": decoded["min_mv"],
                        "max_voltage_mv": decoded["max_mv"],
                        "max_current_ma": decoded["max_ma"],
                        "max_current_15v_ma": decoded["max_ma_15v"],
                        "max_current_20v_ma": decoded["max_ma_20v"],
                        "max_power_mw": decoded["max_mw"],
                        "pdp_w": decoded["pdp_w"],
                        "power_limited": decoded["power_limited"],
                        "peak_current": decoded["peak_current"],
                        "description": decoded["description"],
                    })
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            print(f"Skipping PDOs from {result_path}: {exc}", file=sys.stderr)
    aggregated: dict[tuple[object, ...], dict[str, object]] = {}
    for row in observations:
        key = (
            row["source_id"], row["pdo_region"],
            row["pdo_object_number"], row["pdo_raw_u32"],
        )
        current = aggregated.get(key)
        if current is None:
            current = {
                **row,
                "observation_count": 0,
                "first_captured": row["captured"],
                "last_captured": row["captured"],
                "_cable_ids": set(),
            }
            aggregated[key] = current
        current["observation_count"] = as_int(current["observation_count"]) + 1
        current["first_captured"] = min(
            str(current["first_captured"]), str(row["captured"])
        )
        current["last_captured"] = max(
            str(current["last_captured"]), str(row["captured"])
        )
        if row["cable_id"]:
            cable_ids = current["_cable_ids"]
            if isinstance(cable_ids, set):
                cable_ids.add(str(row["cable_id"]))
    rows = list(aggregated.values())
    for row in rows:
        cable_ids = row.pop("_cable_ids")
        row["observed_cable_ids"] = (
            ";".join(sorted(cable_ids)) if isinstance(cable_ids, set) else ""
        )
    rows.sort(
        key=lambda row: (
            str(row["source_id"]),
            0 if row["pdo_region"] == "SPR" else 1,
            as_int(row["pdo_object_number"]),
        )
    )
    return rows


def write_pdo_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with io.StringIO(newline="") as output:
        writer = csv.DictWriter(output, fieldnames=PDO_CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        atomic_write_text(path, "\ufeff" + output.getvalue())


CABLE_CSV_FIELDS = [
    "cable_id",
    "cable_attachment",
    "owner_source_id",
    "cable_type",
    "current_ma",
    "max_voltage_mv",
    "usb_speed",
    "vid",
    "pid",
    "id_header_vdo",
    "product_vdo",
    "cable_vdo",
    "observation_count",
    "first_captured",
    "last_captured",
    "observed_source_ids",
]


def discover_cable_rows(captures: Path) -> list[dict[str, object]]:
    rows: dict[str, dict[str, object]] = {}
    for result_path in captures.glob("*/session_*/result.json"):
        try:
            data = json.loads(result_path.read_text(encoding="utf-8"))
            captured, _ = capture_details(result_path)
            cable = effective_cable(data)
            source_id, cable_id = capture_ids(result_path, data, cable)
            if not cable_id:
                continue
            attachment = cable_attachment(result_path, data, cable_id)
            row = rows.get(cable_id)
            if row is None:
                cable_type = {3: "Passive", 4: "Active"}.get(
                    as_int(cable.get("cable_type")), "Unknown"
                )
                speed = {
                    0: "USB2 only", 1: "USB3.2 Gen1", 2: "USB4 Gen2",
                    3: "USB4 Gen3", 4: "USB4 Gen4",
                }.get(as_int(cable.get("usb_speed")), "Unknown")
                row = {
                    "cable_id": cable_id,
                    "cable_attachment": "unknown",
                    "owner_source_id": "",
                    "cable_type": cable_type,
                    "current_ma": as_int(cable.get("current_ma")),
                    "max_voltage_mv": as_int(cable.get("max_mv")),
                    "usb_speed": speed,
                    "vid": "0x" + str(cable.get("vid", "0000")).upper(),
                    "pid": "0x" + str(cable.get("pid", "0000")).upper(),
                    "id_header_vdo": "0x" + str(cable.get("id_header", "00000000")).upper(),
                    "product_vdo": "0x" + str(cable.get("product_vdo", "00000000")).upper(),
                    "cable_vdo": "0x" + str(cable.get("cable_vdo", "00000000")).upper(),
                    "observation_count": 0,
                    "first_captured": captured,
                    "last_captured": captured,
                    "_source_ids": set(),
                    "_attachments": set(),
                    "_owner_source_ids": set(),
                }
                rows[cable_id] = row
            row["observation_count"] = as_int(row["observation_count"]) + 1
            row["first_captured"] = min(str(row["first_captured"]), captured)
            row["last_captured"] = max(str(row["last_captured"]), captured)
            source_ids = row["_source_ids"]
            if isinstance(source_ids, set):
                source_ids.add(source_id)
            attachments = row["_attachments"]
            if isinstance(attachments, set):
                attachments.add(attachment)
            owner_source_ids = row["_owner_source_ids"]
            if (
                attachment == "captive"
                and source_id
                and isinstance(owner_source_ids, set)
            ):
                owner_source_ids.add(source_id)
        except (OSError, TypeError, json.JSONDecodeError) as exc:
            print(f"Skipping cable from {result_path}: {exc}", file=sys.stderr)
    result = list(rows.values())
    for row in result:
        source_ids = row.pop("_source_ids")
        attachments = row.pop("_attachments")
        owner_source_ids = row.pop("_owner_source_ids")
        row["observed_source_ids"] = (
            ";".join(sorted(source_ids)) if isinstance(source_ids, set) else ""
        )
        explicit_attachments = (
            attachments - {"unknown"} if isinstance(attachments, set) else set()
        )
        row["cable_attachment"] = (
            next(iter(explicit_attachments))
            if len(explicit_attachments) == 1
            else "mixed" if explicit_attachments else "unknown"
        )
        row["owner_source_id"] = (
            ";".join(sorted(owner_source_ids))
            if isinstance(owner_source_ids, set) else ""
        )
    result.sort(key=lambda row: str(row["cable_id"]))
    return result


def write_cable_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with io.StringIO(newline="") as output:
        writer = csv.DictWriter(output, fieldnames=CABLE_CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        atomic_write_text(path, "\ufeff" + output.getvalue())


def html_cell(
    value: str,
    class_name: str = "",
    *,
    break_after_semicolon: bool = False,
    show_full_value: bool = False,
) -> str:
    class_attr = f' class="{class_name}"' if class_name else ""
    title_attr = f' title="{html.escape(value, quote=True)}"' if show_full_value else ""
    if break_after_semicolon:
        content = ";<br>".join(
            html.escape(part.strip()) for part in re.split(r";\s*", value)
        )
    else:
        content = html.escape(value)
    return f"<td{class_attr}{title_attr}>{content}</td>"


def write_html(path: Path, rows: list[dict[str, str]]) -> None:
    body: list[str] = []
    for row in rows:
        overall = row["Overall"]
        status_class = "pass" if overall == "PASS" else "fail" if overall == "FAIL" else "partial"
        result_status = row["Result Status"].lower()
        result_id = row["Result ID"]
        favorite = row["Favorite"] == "Yes"
        links = (
            f'<a href="{row["_summary"]}">Summary</a> '
            f'<a href="{row["_result"]}">JSON</a> '
            f'<a href="{row["_raw"]}">Raw</a>'
        )
        search = html.escape(" ".join(row[field] for field in CSV_FIELDS).lower(), quote=True)
        cells = [
            (
                '<td class="favorite-cell">'
                f'<button class="favorite" type="button" data-result-id="{html.escape(result_id, quote=True)}" '
                f'aria-pressed="{str(favorite).lower()}" title="{("Remove from" if favorite else "Add to")} favorites">'
                f'{"★" if favorite else "☆"}</button></td>'
            ),
            html_cell(result_id, "result-id", show_full_value=True),
            html_cell(row["Captured"], "compact"),
            html_cell(row["Source ID"], "identifier source-id", show_full_value=True),
            html_cell(row["Cable ID"], "identifier cable-id", show_full_value=True),
            html_cell(row["Cable Attachment"], "compact"),
            html_cell(row["Result Status"], f"result-status {result_status} compact"),
            html_cell(row["Overall"], f"status {status_class} compact"),
            html_cell(row["SPR"], "compact"),
            html_cell(row["PPS"], "compact"),
            html_cell(row["SPR AVS"]),
            html_cell(row["EPR fixed"], "compact"),
            html_cell(row["AVS"], "compact"),
            html_cell(row["PDP"], "compact"),
            html_cell(row["Power/data roles"]),
            html_cell(row["Source flags"], "notes", break_after_semicolon=True),
            html_cell(row["SPR PDO details"], "detail-list", break_after_semicolon=True),
            html_cell(row["SPR PDO raw"], "raw-list", break_after_semicolon=True),
            html_cell(row["EPR PDO details"], "detail-list", break_after_semicolon=True),
            html_cell(row["EPR PDO raw"], "raw-list", break_after_semicolon=True),
            html_cell(row["Source identity"]),
            html_cell(row["Source identity raw"], "raw-list", break_after_semicolon=True),
            html_cell(row["PD revision"], "compact"),
            html_cell(row["Extended identity"]),
            html_cell(row["Source info"]),
            html_cell(row["Manufacturer"]),
            html_cell(row["Probe support"], "detail-list", break_after_semicolon=True),
            html_cell(row["Probe details"], "detail-list", break_after_semicolon=True),
            html_cell(row["Cable"]),
            html_cell(row["Cable observation"], "observation"),
            html_cell(row["Cable VID"], "compact"),
            html_cell(row["Cable raw"], "raw-list", break_after_semicolon=True),
            html_cell(row["SPR contract"], "compact"),
            html_cell(row["PPS probe"], "compact"),
            html_cell(row["EPR enter"], "compact"),
            html_cell(row["Chunking"], "compact"),
            html_cell(row["EPR exit"], "compact"),
            html_cell(row["Detached"], "compact"),
            html_cell(row["Notes"], "detail-list", break_after_semicolon=True),
            f'<td class="files">{links}</td>',
        ]
        body.append(
            f'<tr data-result-status="{html.escape(result_status)}" '
            f'data-favorite="{str(favorite).lower()}" data-search="{search}">'
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
.meta a {{ margin-left: 10px; color: #0969b5; }}
.controls {{ display: flex; flex-wrap: wrap; gap: 8px; align-items: center; padding: 12px 24px; background: #e9edf2; border-bottom: 1px solid #c9ced6; }}
input, button {{ min-height: 34px; padding: 6px 9px; border: 1px solid #aeb6c1; border-radius: 4px; background: #fff; color: #20242a; }}
input {{ width: min(460px, 55vw); }}
.filter-button[aria-pressed="true"] {{ border-color: #1f6f64; background: #dff1ed; color: #12544c; font-weight: 700; }}
.filter-count {{ display: inline-block; min-width: 22px; margin-left: 5px; padding: 1px 5px; border-radius: 10px; background: #e5e9ee; font-size: 11px; text-align: center; }}
.filter-button[aria-pressed="true"] .filter-count {{ background: #fff; }}
#count {{ margin-left: auto; color: #5d6672; font-size: 13px; }}
.table-wrap {{ overflow: auto; height: calc(100vh - 137px); }}
table {{ width: max-content; min-width: 100%; border-collapse: separate; border-spacing: 0; background: #fff; font-size: 13px; }}
th {{ position: sticky; top: 34px; z-index: 2; padding: 8px 9px; text-align: left; white-space: nowrap; background: #343b45; color: #fff; border-right: 1px solid #59616d; }}
thead .group th {{ top: 0; z-index: 3; background: #1f6f64; text-align: center; font-size: 12px; letter-spacing: .02em; }}
td {{ max-width: 270px; padding: 7px 9px; border-right: 1px solid #d9dde3; border-bottom: 1px solid #d9dde3; vertical-align: top; white-space: normal; overflow-wrap: anywhere; word-break: normal; line-height: 1.35; }}
tbody tr:nth-child(even) {{ background: #f7f8fa; }}
tbody tr:hover {{ background: #e8f1fb; }}
.compact {{ white-space: nowrap; overflow-wrap: normal; }}
.identifier {{ min-width: 170px; max-width: 260px; font-weight: 650; overflow-wrap: anywhere; word-break: break-word; }}
.source-id {{ min-width: 180px; }}
.cable-id {{ min-width: 220px; }}
.status {{ font-weight: 750; }}
.status.pass {{ color: #08783e; }}
.status.partial {{ color: #986300; }}
.status.fail {{ color: #b42318; }}
.result-status {{ font-weight: 750; }}
.result-status.valid {{ color: #08783e; }}
.result-status.review {{ color: #986300; }}
.result-status.failed {{ color: #b42318; }}
.favorite-cell {{ padding: 3px 6px; text-align: center; }}
.favorite {{ min-height: 28px; padding: 1px 7px; border-color: transparent; background: transparent; color: #8b6b00; cursor: pointer; font-size: 20px; line-height: 1; }}
.favorite:hover {{ border-color: #c99c00; }}
.favorite[aria-pressed="true"] {{ color: #c28b00; }}
.result-id {{ max-width: 150px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; overflow-wrap: normal; font-family: Consolas, monospace; font-size: 11px; }}
.notes {{ white-space: normal; min-width: 190px; }}
.detail-list {{ min-width: 230px; white-space: normal; }}
.raw-list {{ min-width: 220px; white-space: normal; font-family: Consolas, monospace; font-size: 11px; }}
.observation {{ color: #5d6672; font-style: italic; }}
.files {{ white-space: nowrap; overflow-wrap: normal; }}
.files a {{ margin-right: 8px; color: #0969b5; }}
@media (prefers-color-scheme: dark) {{
  body, table {{ background: #17191d; color: #e6e9ed; }}
  header {{ background: #20242a; border-color: #444b55; }}
  .controls {{ background: #282d34; border-color: #444b55; }}
  input, button {{ background: #17191d; color: #e6e9ed; border-color: #59616d; }}
  .filter-button[aria-pressed="true"] {{ background: #214d47; color: #e6e9ed; }}
  th {{ background: #30363e; }}
  td {{ border-color: #3b414a; }}
  tbody tr:nth-child(even) {{ background: #1d2025; }}
  tbody tr:hover {{ background: #25384a; }}
  .status.pass {{ color: #56d38b; }}
  .status.partial {{ color: #f0bd54; }}
  .status.fail {{ color: #ff8177; }}
  .result-status.valid {{ color: #56d38b; }}
  .result-status.review {{ color: #f0bd54; }}
  .result-status.failed {{ color: #ff8177; }}
  .observation {{ color: #aeb6c1; }}
  .files a {{ color: #75bfff; }}
}}
</style>
</head>
<body>
<header>
  <h1>CH32H417 PD Spec Table</h1>
  <div class="meta">Generated {html.escape(generated)} from {len(rows)} captured session(s). Core PDO/contract fields are negotiated capability data; source detail fields are optional probes; Cable VDO is passive SOP′ observation.
    <a href="spec_table.csv">Session CSV</a>
    <a href="source_table.csv">Source CSV</a>
    <a href="pdo_table.csv">PDO CSV</a>
    <a href="cable_table.csv">Cable CSV</a>
  </div>
</header>
<div class="controls">
  <input id="search" type="search" placeholder="Filter by source ID, cable ID, PDO, VID, status..." aria-label="Filter measurements">
  <button class="filter-button" type="button" data-status-filter="valid" aria-pressed="true">Valid <span class="filter-count" data-filter-count>0</span></button>
  <button class="filter-button" type="button" data-status-filter="failed" aria-pressed="false">Failed <span class="filter-count" data-filter-count>0</span></button>
  <button class="filter-button" type="button" data-status-filter="review" aria-pressed="false">Review <span class="filter-count" data-filter-count>0</span></button>
  <button class="filter-button" type="button" data-status-filter="all" aria-pressed="false">All <span class="filter-count" data-filter-count>0</span></button>
  <button id="favorites" class="filter-button" type="button" aria-pressed="false">★ Favorites <span class="filter-count" id="favorite-count">0</span></button>
  <span id="count"></span>
</div>
<div class="table-wrap">
<table>
<thead>
  <tr class="group">
    <th colspan="6">Capture context</th><th colspan="2">Result</th>
    <th colspan="12">Core negotiated source capabilities</th>
    <th colspan="8">Optional source probes</th>
    <th colspan="4">Passive SOP′ cable observation</th>
    <th colspan="8">Protocol outcome</th>
  </tr>
<tr>
  <th>Favorite</th><th>Result ID</th><th>Captured</th><th>Source ID</th><th>Cable ID</th><th>Cable Attachment</th><th>Result Status</th><th>Overall</th><th>SPR</th><th>PPS</th><th>SPR AVS</th>
  <th>EPR Fixed</th><th>AVS</th><th>PDP</th><th>Power/Data Roles</th>
  <th>Source Flags</th><th>SPR PDO Details</th><th>SPR PDO Raw</th>
  <th>EPR PDO Details</th><th>EPR PDO Raw</th><th>Source Identity</th>
  <th>Source Identity Raw</th>
  <th>PD Revision</th><th>Extended Identity</th><th>Source Info</th>
  <th>Manufacturer</th><th>Probe Support</th><th>Probe Details</th>
  <th>Cable</th><th>Observation</th><th>Cable VID</th><th>Cable Raw</th>
  <th>SPR Contract</th><th>PPS Probe</th>
  <th>EPR Enter</th><th>Chunking</th><th>EPR Exit</th><th>Detached</th>
  <th>Notes</th><th>Files</th>
</tr></thead>
<tbody>{''.join(body)}</tbody>
</table>
</div>
<script>
const search = document.getElementById('search');
const rows = Array.from(document.querySelectorAll('tbody tr'));
const count = document.getElementById('count');
const statusButtons = Array.from(document.querySelectorAll('[data-status-filter]'));
const favorites = document.getElementById('favorites');
const favoriteCount = document.getElementById('favorite-count');
let statusFilter = 'valid';
let favoritesOnly = false;
function filterRows() {{
  const query = search.value.trim().toLowerCase();
  const searchMatches = rows.filter((row) => !query || row.dataset.search.includes(query));
  for (const button of statusButtons) {{
    const filter = button.dataset.statusFilter;
    const matches = filter === 'all'
      ? searchMatches.length
      : searchMatches.filter((row) => row.dataset.resultStatus === filter).length;
    button.setAttribute('aria-pressed', String(filter === statusFilter));
    button.querySelector('[data-filter-count]').textContent = String(matches);
  }}
  const statusMatches = searchMatches.filter((row) =>
    statusFilter === 'all' || row.dataset.resultStatus === statusFilter
  );
  favoriteCount.textContent = String(statusMatches.filter((row) => row.dataset.favorite === 'true').length);
  favorites.setAttribute('aria-pressed', String(favoritesOnly));
  let visible = 0;
  for (const row of rows) {{
    const show = statusMatches.includes(row) && (!favoritesOnly || row.dataset.favorite === 'true');
    row.hidden = !show;
    if (show) visible++;
  }}
  count.textContent = `${{visible}} / ${{rows.length}} sessions`;
}}
search.addEventListener('input', filterRows);
for (const button of statusButtons) {{
  button.addEventListener('click', () => {{
    statusFilter = button.dataset.statusFilter;
    filterRows();
  }});
}}
favorites.addEventListener('click', () => {{
  favoritesOnly = !favoritesOnly;
  filterRows();
}});
for (const button of document.querySelectorAll('.favorite')) {{
  button.addEventListener('click', async () => {{
    if (location.protocol === 'file:') {{
      alert('Start this viewer with pd_report.py --serve to save favorites.');
      return;
    }}
    const favorite = button.getAttribute('aria-pressed') !== 'true';
    button.disabled = true;
    try {{
      const response = await fetch('/api/favorite', {{
        method: 'POST',
        headers: {{'Content-Type': 'application/json'}},
        body: JSON.stringify({{result_id: button.dataset.resultId, favorite}}),
      }});
      if (!response.ok) throw new Error(`HTTP ${{response.status}}`);
      button.setAttribute('aria-pressed', String(favorite));
      button.textContent = favorite ? '★' : '☆';
      button.title = favorite ? 'Remove from favorites' : 'Add to favorites';
      button.closest('tr').dataset.favorite = String(favorite);
      filterRows();
    }} catch (error) {{
      alert(`Could not save favorite: ${{error}}`);
    }} finally {{
      button.disabled = false;
    }}
  }});
}}
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
    source_csv_path = captures / "source_table.csv"
    pdo_csv_path = captures / "pdo_table.csv"
    cable_csv_path = captures / "cable_table.csv"
    html_path = captures / "spec_table.html"
    write_csv(csv_path, rows)
    write_source_csv(source_csv_path, discover_source_rows(rows))
    write_pdo_csv(pdo_csv_path, discover_pdo_rows(captures))
    write_cable_csv(cable_csv_path, discover_cable_rows(captures))
    write_html(html_path, rows)
    try:
        index_rows(captures, rows)
    except (sqlite3.Error, OSError) as exc:
        print(f"Warning: CSV reports saved, but measurement index could not be updated: {exc}", file=sys.stderr)
    return html_path, csv_path, len(rows)


def make_report_handler(captures: Path) -> type[SimpleHTTPRequestHandler]:
    class ReportHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args: object, **kwargs: object) -> None:
            super().__init__(*args, directory=str(captures), **kwargs)

        def send_json(
            self, value: object, status: HTTPStatus = HTTPStatus.OK
        ) -> None:
            body = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def local_client(self) -> bool:
            return self.client_address[0] in {"127.0.0.1", "::1"}

        def do_GET(self) -> None:
            request_path = unquote(urlparse(self.path).path)
            if request_path in {"", "/"}:
                self.send_response(HTTPStatus.FOUND)
                self.send_header("Location", "/spec_table.html")
                self.end_headers()
                return
            if request_path == "/api/annotations":
                self.send_json(load_annotations(captures))
                return
            super().do_GET()

        def do_POST(self) -> None:
            request_path = unquote(urlparse(self.path).path)
            if request_path != "/api/favorite":
                self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                return
            if not self.local_client():
                self.send_json({"error": "localhost only"}, HTTPStatus.FORBIDDEN)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("JSON body must be an object")
                result_id = str(payload.get("result_id") or "")
                favorite = payload.get("favorite")
                if not isinstance(favorite, bool):
                    raise ValueError("favorite must be true or false")
                data = save_favorite(captures, result_id, favorite)
                # Keep generated CSV/HTML as a portable projection of the
                # annotation registry for downstream ASD-PD31 imports.
                generate_reports(captures)
                self.send_json(
                    {
                        "result_id": result_id,
                        "favorite": favorite,
                        "updated": data.get("results", {}).get(result_id, {}),
                    }
                )
            except (ValueError, json.JSONDecodeError) as exc:
                self.send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
            except OSError as exc:
                self.send_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    return ReportHandler


class ExclusiveThreadingHTTPServer(ThreadingHTTPServer):
    """Prevent Windows SO_REUSEADDR from sharing a viewer port silently."""

    allow_reuse_address = False

    def server_bind(self) -> None:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def serve_reports(captures: Path, host: str, port: int) -> None:
    try:
        server = ExclusiveThreadingHTTPServer(
            (host, port), make_report_handler(captures)
        )
    except OSError as exc:
        raise RuntimeError(
            f"Could not start CH32H417 results viewer on {host}:{port}; "
            "the port is already in use. Stop the other viewer or choose "
            f"another port with --port (for example, --port {port + 1})."
        ) from exc
    print(f"CH32H417 results viewer: http://{host}:{port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping results viewer.")
    finally:
        server.server_close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  python tools/pd_report.py --history --source-id adapter-01
  python tools/pd_report.py --history --cable-id cable-01 --limit 100
  python tools/pd_report.py --serve --port 8766

Rebuilds CSV/HTML exports and the SQLite session index from old and new captures.
Preserves saved profiles, result IDs, and favorites; no hardware is required.
Point the dashboard's ch32_capture_root at this folder containing spec_table.csv.
Back up the entire captures folder, including measurements.sqlite3 (profiles)
and result_annotations.json (favorites), as well as the original logs and JSON.
""",
    )
    parser.add_argument("--captures", type=Path, default=DEFAULT_CAPTURES)
    parser.add_argument("--serve", action="store_true", help="serve the report and enable persistent favorites")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--history", action="store_true", help="print indexed measurement history after refreshing reports")
    parser.add_argument("--source-id", default="", help="filter --history by physical source ID")
    parser.add_argument("--cable-id", default="", help="filter --history by physical cable ID")
    parser.add_argument("--limit", type=int, default=50, help="maximum history rows")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    html_path, csv_path, count = generate_reports(args.captures.resolve())
    print(f"Wrote {html_path}")
    print(f"Wrote {csv_path}")
    print(f"Wrote {args.captures.resolve() / 'source_table.csv'}")
    print(f"Wrote {args.captures.resolve() / 'pdo_table.csv'}")
    print(f"Wrote {args.captures.resolve() / 'cable_table.csv'}")
    print(f"Sessions: {count}")
    print(f"Index: {args.captures.resolve() / DATABASE_NAME}")
    if args.history:
        for row in history(args.captures.resolve(), args.source_id, args.cable_id, args.limit):
            print(f"{row['Captured']}\t{row['Source ID']}\t{row['Cable ID']}\t{row['Result Status']}\t{row['Result ID']}\t{row.get('Firmware Version', '')}\t{row.get('Test Note', '')}")
    if args.serve:
        try:
            serve_reports(args.captures.resolve(), args.host, args.port)
        except RuntimeError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
