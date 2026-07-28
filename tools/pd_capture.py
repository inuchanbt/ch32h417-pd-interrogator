#!/usr/bin/env python3
"""Capture and summarize CH32H417 PD interrogator UART records."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import BinaryIO, Iterable, TextIO

from pd_report import DEFAULT_CAPTURES, generate_reports

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised only on missing dependency
    serial = None
    list_ports = None


RECORD_PREFIX = "@PD1,"
KEY_RE = re.compile(r"^[a-z][a-z0-9_]*$")
CABLE_VDO_RE = re.compile(r"^\s*(Passive|Active) Cable VDO1:0x([0-9A-Fa-f]{8})\s*$")
EXT_PAYLOAD_BYTE_RE = re.compile(r"\[(\d+)\]=([0-9A-Fa-f]{2})")
WRITE_RETRY_COUNT = 40
WRITE_RETRY_DELAY_S = 0.05


def atomic_write_text(path: Path, text: str, encoding: str = "utf-8") -> None:
    """Replace a text file atomically, tolerating short Windows file locks."""
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.{time.monotonic_ns()}.tmp"
    )
    temporary.write_text(text, encoding=encoding)
    try:
        for attempt in range(WRITE_RETRY_COUNT):
            try:
                os.replace(temporary, path)
                return
            except PermissionError:
                if attempt + 1 >= WRITE_RETRY_COUNT:
                    raise
                time.sleep(WRITE_RETRY_DELAY_S)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        except OSError:
            # A stale hidden temporary is preferable to losing the capture.
            pass


def parse_record(line: str) -> dict[str, str] | None:
    """Extract one @PD1 key/value record, tolerating debug text before it."""
    start = line.find(RECORD_PREFIX)
    if start < 0:
        return None
    payload = line[start + len(RECORD_PREFIX) :].strip()
    record: dict[str, str] = {"format": "PD1"}
    for item in payload.split(","):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        key = key.strip()
        value = value.strip().split()[0] if value.strip() else ""
        if KEY_RE.match(key):
            record[key] = value
    if "type" not in record or "session" not in record:
        return None
    return record


def as_int(value: str | None, default: int = 0, base: int = 10) -> int:
    if value is None or value == "":
        return default
    try:
        return int(value, base)
    except ValueError:
        return default


def format_power(mw: int) -> str:
    if mw <= 0:
        return "unknown"
    if mw % 1000 == 0:
        return f"{mw // 1000}W"
    return f"{mw / 1000:.1f}W"


def format_voltage(mv: int) -> str:
    if mv % 1000 == 0:
        return f"{mv // 1000}V"
    return f"{mv / 1000:.1f}V"


def display_status(value: str | None) -> str:
    names = {
        "pass": "PASS",
        "fail": "FAIL",
        "na": "N/A",
        "pending": "PENDING",
        "unsupported": "UNSUPPORTED",
        "no_response": "NO RESPONSE",
        "tx_failed": "TX FAILED",
        "nak": "NAK",
        "busy": "BUSY",
        "rejected": "REJECTED",
        "invalid": "INVALID",
        "unknown": "UNKNOWN",
    }
    return names.get(value or "unknown", (value or "unknown").upper())


def decode_cable_vdo_from_log(line: str) -> dict[str, str] | None:
    """Recover a passively sniffed SOP' Cable VDO before the MCU emits @PD1."""
    match = CABLE_VDO_RE.match(line)
    if match is None:
        return None
    cable_type_name, raw = match.groups()
    vdo = int(raw, 16)
    current_code = (vdo >> 5) & 0x03
    current_ma = {1: 3000, 2: 5000}.get(current_code, 0)
    voltage_code = (vdo >> 9) & 0x03
    max_mv = (20000, 30000, 40000, 48000)[voltage_code]
    speed_code = vdo & 0x07
    speed_name = {
        0: "usb2", 1: "usb3_gen1", 2: "usb4_gen2",
        3: "usb4_gen3", 4: "usb4_gen4",
    }.get(speed_code, "unknown")
    cable_type = "3" if cable_type_name == "Passive" else "4"
    return {
        "format": "PD1",
        "type": "cable",
        "status": "pass",
        "captured_via": "passive_raw_log",
        "cable_type": cable_type,
        "cable_type_name": cable_type_name.lower(),
        "current_ma": str(current_ma),
        "cable_current_a": str(current_ma // 1000),
        "max_mv": str(max_mv),
        "cable_max_voltage_v": str(max_mv // 1000),
        "usb_speed": str(speed_code),
        "usb_speed_name": speed_name,
        "cable_vdo_version": str((vdo >> 21) & 0x07),
        "cable_latency": str((vdo >> 13) & 0x0F),
        "cable_termination": str((vdo >> 11) & 0x03),
        "cable_vdo": raw.upper(),
    }


def decode_source_cap_ext_from_log(line: str) -> dict[str, str] | None:
    """Decode the 24/25-byte Source Capabilities Extended payload in raw logs."""
    if "EXT payload bytes:" not in line:
        return None
    payload: dict[int, int] = {
        int(index): int(value, 16)
        for index, value in EXT_PAYLOAD_BYTE_RE.findall(line)
    }
    if not all(index in payload for index in range(24)):
        return None
    size = max(payload) + 1
    return {
        "format": "PD1",
        "type": "source_cap_ext",
        "captured_via": "passive_raw_log",
        "size": str(size),
        "vid": f"{payload[0] | (payload[1] << 8):04X}",
        "pid": f"{payload[2] | (payload[3] << 8):04X}",
        "xid": f"{payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24):08X}",
        "fw_version": str(payload[8]),
        "hw_version": str(payload[9]),
        "voltage_regulation_raw": str(payload[10]),
        "hold_up_time_raw": str(payload[11]),
        "compliance_raw": str(payload[12]),
        "touch_current_raw": str(payload[13]),
        "peak_current_1_raw": str(payload[14] | (payload[15] << 8)),
        "peak_current_2_raw": str(payload[16] | (payload[17] << 8)),
        "peak_current_3_raw": str(payload[18] | (payload[19] << 8)),
        "touch_temperature_raw": str(payload[20]),
        "source_inputs_raw": str(payload[21]),
        "fixed_batteries": str(payload[22] & 0x0F),
        "swappable_batteries": str((payload[22] >> 4) & 0x0F),
        "spr_pdp_w": str(payload[23]),
        "epr_pdp_w": str(payload.get(24, 0)),
    }


@dataclass
class SessionState:
    session: int
    firmware_session: int = 0
    records: list[dict[str, str]] = field(default_factory=list)
    attach: dict[str, str] = field(default_factory=dict)
    spr: dict[str, str] = field(default_factory=dict)
    spr_pdos: dict[int, str] = field(default_factory=dict)
    epr: dict[str, str] = field(default_factory=dict)
    epr_fixed: dict[int, tuple[int, int]] = field(default_factory=dict)
    epr_pdos: dict[int, str] = field(default_factory=dict)
    identity: dict[str, str] = field(default_factory=dict)
    cable: dict[str, str] = field(default_factory=dict)
    probe_status: dict[str, str] = field(default_factory=dict)
    probe_details: dict[str, dict[str, str]] = field(default_factory=dict)
    result: dict[str, str] = field(default_factory=dict)
    detached: bool = False

    def apply(self, record: dict[str, str]) -> None:
        self.records.append(record)
        kind = record.get("type")
        if kind == "attach":
            self.attach.update(record)
        elif kind == "spr":
            self.spr.update(record)
        elif kind == "spr_pdo":
            raw = record.get("raw", "")
            if re.fullmatch(r"[0-9A-Fa-f]{8}", raw):
                self.spr_pdos[as_int(record.get("index"))] = raw.upper()
        elif kind == "epr":
            self.epr.update(record)
        elif kind == "epr_fixed":
            self.epr_fixed[as_int(record.get("index"))] = (
                as_int(record.get("mv")),
                as_int(record.get("ma")),
            )
        elif kind == "epr_pdo":
            raw = record.get("raw", "")
            if re.fullmatch(r"[0-9A-Fa-f]{8}", raw):
                self.epr_pdos[as_int(record.get("index"))] = raw.upper()
        elif kind == "identity":
            self.identity.update(record)
        elif kind == "cable":
            # Older firmware emits an all-zero/unknown cable record at detach
            # even when an early passive SOP' observation was recovered from
            # raw.log. Never discard the stronger observation.
            if self.cable.get("status") == "pass" and record.get("status") != "pass":
                return
            self.cable.update(record)
        elif kind == "probe":
            name = record.get("name", "unknown")
            self.probe_status[name] = record.get("status", "unknown")
        elif kind in {
            "revision",
            "source_cap_ext",
            "source_status",
            "source_info",
            "pps_status",
            "country_codes",
            "manufacturer",
            "battery_cap",
            "battery_status",
        }:
            self.probe_details[kind] = record.copy()
        elif kind == "result":
            self.result.update(record)
        elif kind == "detach":
            self.detached = True

    def overall(self) -> str:
        spr = self.result.get("spr", "unknown")
        epr_capable = as_int(self.spr.get("epr_capable")) == 1
        epr = self.result.get("epr", "unknown")
        chunk = self.result.get("chunk", "unknown")
        if spr == "fail" or epr == "fail":
            return "FAIL"
        if chunk == "fail":
            return "PARTIAL" if self.epr_pdos else "FAIL"
        if spr != "pass":
            return "PARTIAL"
        if epr_capable and (epr != "pass" or chunk != "pass"):
            return "PARTIAL"
        return "PASS"

    def protocol_measurement_ready(self) -> bool:
        if self.result.get("spr") != "pass":
            return False
        if as_int(self.spr.get("epr_capable")) == 1:
            return (
                self.result.get("epr") == "pass"
                and self.result.get("chunk")
                in {"pass", "fail", "no_response", "tx_failed", "unsupported"}
            )
        return self.result.get("pps") not in {"pending", "unknown"}

    def render_summary(self) -> str:
        pdo_count = as_int(self.spr.get("pdo_count"))
        pps_count = as_int(self.spr.get("pps_count"))
        max_mw = as_int(self.spr.get("max_mw"))
        pps_text = "PPS none" if pps_count == 0 else f"PPS {pps_count} APDO"
        source_lines = [
            "[Source Summary]",
            f"SPR : {pdo_count} PDO / {pps_text} / Max {format_power(max_mw)}",
        ]

        if as_int(self.spr.get("epr_capable")) == 0:
            source_lines.append("EPR : not advertised")
        elif as_int(self.epr.get("complete")) == 1:
            fixed = [format_voltage(mv) for _, (mv, _) in sorted(self.epr_fixed.items()) if mv]
            epr_parts = [", ".join(fixed) if fixed else "no fixed EPR PDO"]
            avs_min = as_int(self.epr.get("avs_min_mv"))
            avs_max = as_int(self.epr.get("avs_max_mv"))
            pdp = as_int(self.epr.get("pdp_w"))
            if avs_min and avs_max:
                epr_parts.append(f"AVS {format_voltage(avs_min)}-{format_voltage(avs_max)}")
            if pdp:
                epr_parts.append(f"PDP {pdp}W")
            source_lines.append("EPR : " + " / ".join(epr_parts))
        else:
            source_lines.append("EPR : advertised / data unavailable")

        identity_status = self.identity.get("status", "unknown")
        if identity_status == "pass":
            vid = as_int(self.identity.get("vid"), base=16)
            pid = as_int(self.identity.get("pid"), base=16)
            if vid == 0 and pid == 0:
                identity_text = "ACK, VID/PID unavailable"
            else:
                identity_text = f"ACK, VID 0x{vid:04X} / PID 0x{pid:04X}"
        else:
            identity_text = display_status(identity_status)
        source_lines.append(f"Identity : {identity_text}")

        detail_lines: list[str] = []
        revision = self.probe_details.get("revision", {})
        if revision:
            detail_lines.extend(
                [
                    "",
                    "[Source Details]",
                    "Revision : PD "
                    f"{revision.get('pd_major', '?')}.{revision.get('pd_minor', '?')} / "
                    f"USB {revision.get('usb_major', '?')}.{revision.get('usb_minor', '?')}",
                ]
            )
        source_cap_ext = self.probe_details.get("source_cap_ext", {})
        if source_cap_ext:
            if not detail_lines:
                detail_lines.extend(["", "[Source Details]"])
            ext_parts = [
                f"VID 0x{source_cap_ext.get('vid', '0000')}",
                f"PID 0x{source_cap_ext.get('pid', '0000')}",
                f"XID 0x{source_cap_ext.get('xid', '00000000')}",
            ]
            if source_cap_ext.get("fw_version") or source_cap_ext.get("hw_version"):
                ext_parts.append(
                    f"FW {source_cap_ext.get('fw_version', '?')} / "
                    f"HW {source_cap_ext.get('hw_version', '?')}"
                )
            if source_cap_ext.get("spr_pdp_w"):
                pdp = f"SPR PDP {source_cap_ext['spr_pdp_w']}W"
                if source_cap_ext.get("epr_pdp_w"):
                    pdp += f" / EPR PDP {source_cap_ext['epr_pdp_w']}W"
                ext_parts.append(pdp)
            detail_lines.append("Source Cap Ext : " + " / ".join(ext_parts))
        source_info = self.probe_details.get("source_info", {})
        if source_info:
            if not detail_lines:
                detail_lines.extend(["", "[Source Details]"])
            input_name = {"0": "None", "1": "AC", "2": "DC", "3": "AC+DC"}.get(
                source_info.get("present_input", ""), "Unknown"
            )
            detail_lines.append(
                f"Source Info : PDP {source_info.get('pdp_w', '?')}W / "
                f"Temp raw {source_info.get('temp_raw', '?')} / Input {input_name}"
            )
        manufacturer = self.probe_details.get("manufacturer", {})
        if manufacturer:
            if not detail_lines:
                detail_lines.extend(["", "[Source Details]"])
            name = manufacturer.get("name", "unavailable").replace("_", " ")
            detail_lines.append(
                f"Manufacturer : {name} / VID 0x{manufacturer.get('vid', '0000')} / "
                f"PID 0x{manufacturer.get('pid', '0000')}"
            )
        if self.probe_status:
            if not detail_lines:
                detail_lines.extend(["", "[Source Details]"])
            probe_names = {
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
            support = [
                f"{probe_names.get(name, name)}={display_status(status)}"
                for name, status in self.probe_status.items()
            ]
            detail_lines.append("Probes : " + " / ".join(support))

        cable_lines = ["", "[Cable Summary]"]
        if self.cable.get("status") == "pass":
            cable_types = {3: "Passive", 4: "Active"}
            speeds = {
                0: "USB2 only",
                1: "USB3.2 Gen1",
                2: "USB4 Gen2",
                3: "USB4 Gen3",
                4: "USB4 Gen4",
            }
            ctype = cable_types.get(as_int(self.cable.get("cable_type")), "Unknown")
            current_ma = as_int(self.cable.get("current_ma"))
            max_mv = as_int(self.cable.get("max_mv"))
            speed = speeds.get(as_int(self.cable.get("usb_speed")), "Unknown")
            current = f"{current_ma // 1000}A" if current_ma else "default current"
            cable_lines.append(
                f"{ctype} / {current} / {format_voltage(max_mv)} / {speed}"
            )
            cable_lines.append(f"VID : 0x{as_int(self.cable.get('vid'), base=16):04X}")
        else:
            cable_lines.append("Identity : not captured (passive best-effort)")

        exit_names = {
            "ack": "ACK",
            "soft_reset": "Soft Reset",
            "source_cap": "Source Capabilities",
            "timeout": "Timeout",
            "unexpected": "Unexpected response",
            "unknown": "N/A",
        }
        protocol_lines = [
            "",
            "[Protocol Result]",
            f"SPR Contract : {display_status(self.result.get('spr'))}",
            f"PPS Probe    : {display_status(self.result.get('pps'))}",
            f"EPR Enter    : {display_status(self.result.get('epr'))}",
            f"Chunking     : {display_status(self.result.get('chunk'))}",
            f"EPR Exit     : {exit_names.get(self.result.get('exit', 'unknown'), 'N/A')}",
            f"Overall      : {self.overall()}",
        ]
        return "\n".join(source_lines + detail_lines + cable_lines + protocol_lines) + "\n"

    def as_json(self) -> dict[str, object]:
        return {
            "session": self.session,
            "firmware_session": self.firmware_session,
            "attach": self.attach,
            "source": {"summary": self.spr, "pdos": self.spr_pdos},
            "epr": {
                "summary": self.epr,
                "fixed": self.epr_fixed,
                "pdos": self.epr_pdos,
            },
            "identity": self.identity,
            "cable": self.cable,
            "probes": {
                "status": self.probe_status,
                "details": self.probe_details,
            },
            "protocol": self.result,
            "overall": self.overall(),
            "detached": self.detached,
        }


class CaptureRun:
    def __init__(
        self,
        output: Path,
        source_id: str = "",
        cable_id: str = "",
        cable_attachment: str = "unknown",
        source_manufacturer: str = "",
        source_model: str = "",
        source_port: str = "",
        cable_manufacturer: str = "",
        cable_model: str = "",
        cable_length_m: float | None = None,
        input_ac_voltage_v: float = 100.0,
        input_ac_frequency_hz: float = 50.0,
        verbose: bool = False,
    ) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        source_id = source_id.strip()
        cable_id = cable_id.strip()
        source_manufacturer = source_manufacturer.strip()
        source_model = source_model.strip()
        if cable_attachment == "captive" and not cable_id and source_id:
            cable_id = f"{source_id}:captive"
        source_folder_parts = []
        for value in (source_manufacturer, source_model):
            safe_value = re.sub(r"[^A-Za-z0-9_.-]+", "-", value).strip("-")
            if safe_value:
                source_folder_parts.append(safe_value)
        suffix = "_" + "_".join(source_folder_parts) if source_folder_parts else ""
        self.root = output / f"{stamp}{suffix}"
        serial_number = 1
        while self.root.exists():
            self.root = output / f"{stamp}{suffix}_{serial_number:02d}"
            serial_number += 1
        self.root.mkdir(parents=True, exist_ok=False)
        self.source_id = source_id
        self.cable_id = cable_id
        self.cable_attachment = cable_attachment
        self.setup_metadata = {
            "source_manufacturer": source_manufacturer,
            "source_model": source_model,
            "source_port": source_port.strip(),
            "cable_manufacturer": cable_manufacturer.strip(),
            "cable_model": cable_model.strip(),
            "cable_length_m": cable_length_m,
            "input_ac_voltage_v": input_ac_voltage_v,
            "input_ac_frequency_hz": input_ac_frequency_hz,
        }
        (self.root / "capture.json").write_text(
            json.dumps(
                {
                    "format": "PD_CAPTURE_1",
                    "source_id": source_id,
                    "cable_id": cable_id,
                    "cable_attachment": cable_attachment,
                    **self.setup_metadata,
                },
                indent=2,
                sort_keys=True,
            ) + "\n",
            encoding="utf-8",
        )
        self.verbose = verbose
        self.raw_binary: BinaryIO = (self.root / "raw.bin").open("wb")
        self.raw_text: TextIO = (self.root / "raw.log").open("w", encoding="utf-8", newline="")
        self.events: TextIO = (self.root / "events.jsonl").open("w", encoding="utf-8")
        self.sessions: dict[int, SessionState] = {}
        self.session_files: dict[int, tuple[Path, TextIO, TextIO]] = {}
        self.pending_passive_cable: dict[str, str] | None = None
        self.pending_source_cap_ext_session: int | None = None
        self.current_session: int | None = None
        self.next_session = 0
        self.force_new_session = False
        self.last_shown_summary: dict[int, str] = {}
        self.buffer = bytearray()
        self.pending_lines: list[str] = []
        self.closed = False

    def feed(self, data: bytes) -> None:
        self.raw_binary.write(data)
        self.raw_binary.flush()
        self.buffer.extend(data)
        while True:
            cr = self.buffer.find(b"\r")
            lf = self.buffer.find(b"\n")
            separators = [index for index in (cr, lf) if index >= 0]
            if not separators:
                break
            line_end = min(separators)
            terminator_end = line_end + 1
            if (
                self.buffer[line_end] == 0x0D
                and terminator_end < len(self.buffer)
                and self.buffer[terminator_end] == 0x0A
            ):
                terminator_end += 1
            raw_line = bytes(self.buffer[:terminator_end])
            del self.buffer[:terminator_end]
            self.process_line(raw_line)

    def process_line(self, raw_line: bytes) -> None:
        line = raw_line.decode("utf-8", errors="replace")
        self.raw_text.write(line)
        self.raw_text.flush()
        if self.verbose:
            print(line, end="")

        passive_cable = decode_cable_vdo_from_log(line)
        if passive_cable is not None:
            self.pending_passive_cable = passive_cable
        source_cap_ext = decode_source_cap_ext_from_log(line)
        if source_cap_ext is not None and self.pending_source_cap_ext_session is not None:
            state = self.sessions.get(self.pending_source_cap_ext_session)
            if state is not None:
                source_cap_ext["session"] = str(state.firmware_session)
                state.apply(source_cap_ext)
                self._save_session(state)
            self.pending_source_cap_ext_session = None

        record = parse_record(line)
        if record is not None and record.get("type") == "boot":
            if self.current_session in self.sessions:
                self._save_session(self.sessions[self.current_session])
            self.current_session = None
            self.force_new_session = True
            self._hold_pending(line)
            self._write_global_event(record, None)
            return

        if record is None:
            self._write_session_line(line)
            return

        firmware_session = as_int(record.get("session"))
        if firmware_session <= 0:
            self._write_session_line(line)
            return
        kind = record.get("type")
        state, started = self._route_session(record, firmware_session)
        if started:
            session_raw = self.session_files[state.session][1]
            session_raw.writelines(self.pending_lines)
            self.pending_lines.clear()
        self.session_files[state.session][1].write(line)
        self.session_files[state.session][1].flush()
        state.apply(record)
        if kind == "source_cap_ext" or (
            kind == "probe"
            and record.get("name") == "source_cap_ext"
            and record.get("status") == "pass"
        ):
            self.pending_source_cap_ext_session = state.session
        if started and kind == "attach" and self.pending_passive_cable is not None:
            recovered = {
                **self.pending_passive_cable,
                "session": str(firmware_session),
            }
            state.apply(recovered)
            self.pending_passive_cable = None
        self._write_global_event(record, state.session)
        envelope = self._event_envelope(record, state.session)
        self.session_files[state.session][2].write(
            json.dumps(envelope, ensure_ascii=True, sort_keys=True) + "\n"
        )
        self.session_files[state.session][2].flush()
        self._save_session(state)

        if kind == "result" and record.get("complete") == "1":
            self._show_summary(state, "measurement complete")
        elif kind == "result" and state.protocol_measurement_ready():
            self._show_summary(state, "protocol complete; identity pending")
        elif kind == "cable" and state.result.get("complete") == "1":
            self._show_summary(state, "cable update")
        elif kind == "detach":
            self._show_summary(state, "detached")
            self.current_session = None

    def _hold_pending(self, line: str) -> None:
        self.pending_lines.append(line)
        if len(self.pending_lines) > 2000:
            self.pending_lines.pop(0)

    def _write_session_line(self, line: str) -> None:
        if self.current_session in self.session_files:
            raw = self.session_files[self.current_session][1]
            raw.write(line)
            raw.flush()
        else:
            self._hold_pending(line)

    def _route_session(
        self, record: dict[str, str], firmware_session: int
    ) -> tuple[SessionState, bool]:
        kind = record.get("type")
        current = self.sessions.get(self.current_session or -1)
        needs_new = current is None
        if current is not None and current.firmware_session != firmware_session:
            needs_new = True
        if kind == "attach" and self.force_new_session:
            needs_new = True
        if needs_new:
            state = self._start_session(firmware_session)
            self.current_session = state.session
            self.force_new_session = False
            return state, True
        return current, False

    def _start_session(self, firmware_session: int) -> SessionState:
        self.next_session += 1
        host_session = self.next_session
        state = SessionState(host_session, firmware_session)
        folder = self.root / f"session_{host_session:04d}"
        folder.mkdir(parents=True, exist_ok=False)
        raw = (folder / "raw.log").open("w", encoding="utf-8", newline="")
        events = (folder / "events.jsonl").open("w", encoding="utf-8")
        self.sessions[host_session] = state
        self.session_files[host_session] = (folder, raw, events)
        return state

    @staticmethod
    def _event_envelope(record: dict[str, str], host_session: int | None) -> dict[str, object]:
        return {
            "host_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "host_session": host_session,
            **record,
        }

    def _write_global_event(
        self, record: dict[str, str], host_session: int | None
    ) -> None:
        encoded = json.dumps(
            self._event_envelope(record, host_session),
            ensure_ascii=True,
            sort_keys=True,
        )
        self.events.write(encoded + "\n")
        self.events.flush()

    def _save_session(self, state: SessionState) -> None:
        folder = self.session_files[state.session][0]
        result = state.as_json()
        result["source_id"] = self.source_id
        result["cable_id"] = self.cable_id
        result["cable_attachment"] = self.cable_attachment
        result.update(self.setup_metadata)
        files = (
            (folder / "summary.txt", state.render_summary()),
            (
                folder / "result.json",
                json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True)
                + "\n",
            ),
        )
        for path, contents in files:
            try:
                atomic_write_text(path, contents)
            except OSError as exc:
                print(
                    f"Warning: could not update session {state.session} "
                    f"{path.name}; keeping the previous saved version: {exc}",
                    file=sys.stderr,
                )

    def _show_summary(self, state: SessionState, reason: str) -> None:
        summary = state.render_summary()
        if self.last_shown_summary.get(state.session) == summary:
            if reason == "detached":
                print(f"\nSession {state.session} finalized: {state.overall()}")
            return
        self.last_shown_summary[state.session] = summary
        firmware = (
            f", firmware {state.firmware_session}"
            if state.firmware_session != state.session
            else ""
        )
        print(f"\n=== Session {state.session}{firmware} ({reason}) ===")
        print(summary, end="")

    def close(self) -> None:
        if self.closed:
            return
        try:
            if self.buffer:
                self.process_line(bytes(self.buffer))
                self.buffer.clear()
            for state in self.sessions.values():
                self._save_session(state)
        finally:
            for _, raw, events in self.session_files.values():
                raw.close()
                events.close()
            self.raw_binary.close()
            self.raw_text.close()
            self.events.close()
            self.closed = True

        # Capture artifacts are durable before rebuilding the aggregate reports.
        try:
            html_path, csv_path, count = generate_reports(self.root.parent)
            print(f"Spec table updated: {html_path} ({count} sessions)")
            print(f"CSV updated: {csv_path}")
            print(f"Source CSV updated: {self.root.parent / 'source_table.csv'}")
            print(f"PDO CSV updated: {self.root.parent / 'pdo_table.csv'}")
            print(f"Cable CSV updated: {self.root.parent / 'cable_table.csv'}")
        except KeyboardInterrupt:
            print(
                "Report update interrupted; capture files were saved successfully.",
                file=sys.stderr,
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            print(f"Warning: could not update spec table: {exc}", file=sys.stderr)


def available_ports() -> list[object]:
    if list_ports is None:
        return []
    return list(list_ports.comports())


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    ports = available_ports()
    preferred = [
        port for port in ports
        if getattr(port, "vid", None) == 0x1A86
        or "WCH" in (getattr(port, "description", "") or "").upper()
    ]
    candidates = preferred if len(preferred) == 1 else ports
    if len(candidates) == 1:
        return str(candidates[0].device)
    details = "\n".join(
        f"  {port.device}: {port.description}"
        for port in ports
    ) or "  (no serial ports found)"
    raise SystemExit("Specify --port. Available ports:\n" + details)


def replay(path: Path, capture: CaptureRun) -> None:
    with path.open("rb") as source:
        while data := source.read(4096):
            capture.feed(data)


def capture_serial(port: str, baud: int, capture: CaptureRun) -> None:
    if serial is None:
        raise SystemExit("pyserial is required: python -m pip install pyserial")
    print(f"Capturing {port} at {baud} baud. Press Ctrl+C to stop.")
    reconnecting = False
    while True:
        try:
            with serial.Serial(port=port, baudrate=baud, timeout=0.1) as device:
                if reconnecting:
                    print(f"Serial reconnected: {port}")
                    reconnecting = False
                while True:
                    data = device.read(4096)
                    if data:
                        capture.feed(data)
        except (serial.SerialException, OSError) as exc:
            if not reconnecting:
                print(
                    f"Serial connection lost ({exc}); retrying {port}. "
                    "Press Ctrl+C to stop.",
                    file=sys.stderr,
                )
            reconnecting = True
            time.sleep(0.5)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port (auto-selected when unambiguous)")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output", type=Path, default=DEFAULT_CAPTURES)
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument("--source-id", default="", help="stable source identifier")
    source_group.add_argument(
        "--label", dest="source_id", help="deprecated alias for --source-id"
    )
    parser.add_argument("--cable-id", default="", help="stable physical cable identifier")
    parser.add_argument(
        "--cable-attachment",
        choices=("unknown", "detachable", "captive"),
        default="unknown",
        help="whether the cable is replaceable or permanently attached",
    )
    parser.add_argument("--source-manufacturer", default="", help="source manufacturer")
    parser.add_argument("--source-model", default="", help="source model")
    parser.add_argument(
        "--source-port",
        default="",
        help="source connection port, for example C1",
    )
    parser.add_argument("--cable-manufacturer", default="", help="cable manufacturer")
    parser.add_argument("--cable-model", default="", help="cable model")
    parser.add_argument(
        "--cable-length-m",
        type=float,
        default=None,
        help="cable length in metres",
    )
    parser.add_argument(
        "--input-ac-voltage",
        "--input-ac-voltage-v",
        dest="input_ac_voltage_v",
        type=float,
        default=100.0,
        help="input AC voltage in volts (default: 100)",
    )
    parser.add_argument(
        "--input-ac-frequency",
        "--input-ac-frequency-hz",
        dest="input_ac_frequency_hz",
        type=float,
        default=50.0,
        help="input AC frequency in hertz (default: 50)",
    )
    parser.add_argument("--verbose", action="store_true", help="echo all firmware debug lines")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--replay", type=Path, help="parse a saved UART log instead of a COM port")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.input_ac_voltage_v <= 0:
        parser.error("--input-ac-voltage must be greater than zero")
    if args.input_ac_frequency_hz <= 0:
        parser.error("--input-ac-frequency must be greater than zero")
    if args.cable_length_m is not None and args.cable_length_m < 0:
        parser.error("--cable-length-m must be zero or greater")
    if args.list:
        for port in available_ports():
            print(f"{port.device}\t{port.description}\tVID={getattr(port, 'vid', None)!r}")
        return 0

    capture = CaptureRun(
        args.output,
        source_id=args.source_id,
        cable_id=args.cable_id,
        cable_attachment=args.cable_attachment,
        source_manufacturer=args.source_manufacturer,
        source_model=args.source_model,
        source_port=args.source_port,
        cable_manufacturer=args.cable_manufacturer,
        cable_model=args.cable_model,
        cable_length_m=args.cable_length_m,
        input_ac_voltage_v=args.input_ac_voltage_v,
        input_ac_frequency_hz=args.input_ac_frequency_hz,
        verbose=args.verbose,
    )
    print(f"Capture directory: {capture.root.resolve()}")
    try:
        if args.replay:
            replay(args.replay, capture)
        else:
            capture_serial(choose_port(args.port), args.baud, capture)
    except KeyboardInterrupt:
        print("\nCapture stopped.")
    finally:
        try:
            capture.close()
        except KeyboardInterrupt:
            print(
                "Capture shutdown interrupted after closing available files.",
                file=sys.stderr,
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
