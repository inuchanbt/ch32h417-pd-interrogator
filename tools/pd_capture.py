#!/usr/bin/env python3
"""Capture and summarize CH32H417 PD interrogator UART records."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import BinaryIO, Iterable, TextIO

from pd_report import generate_reports

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised only on missing dependency
    serial = None
    list_ports = None


RECORD_PREFIX = "@PD1,"
KEY_RE = re.compile(r"^[a-z][a-z0-9_]*$")


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
            detail_lines.append(
                "Source Cap Ext : "
                f"VID 0x{source_cap_ext.get('vid', '0000')} / "
                f"PID 0x{source_cap_ext.get('pid', '0000')} / "
                f"XID 0x{source_cap_ext.get('xid', '00000000')}"
            )
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
    def __init__(self, output: Path, label: str = "", verbose: bool = False) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = "_" + re.sub(r"[^A-Za-z0-9_.-]+", "-", label).strip("-") if label else ""
        self.root = output / f"{stamp}{suffix}"
        serial_number = 1
        while self.root.exists():
            self.root = output / f"{stamp}{suffix}_{serial_number:02d}"
            serial_number += 1
        self.root.mkdir(parents=True, exist_ok=False)
        self.verbose = verbose
        self.raw_binary: BinaryIO = (self.root / "raw.bin").open("wb")
        self.raw_text: TextIO = (self.root / "raw.log").open("w", encoding="utf-8", newline="")
        self.events: TextIO = (self.root / "events.jsonl").open("w", encoding="utf-8")
        self.sessions: dict[int, SessionState] = {}
        self.session_files: dict[int, tuple[Path, TextIO, TextIO]] = {}
        self.current_session: int | None = None
        self.next_session = 0
        self.force_new_session = False
        self.last_shown_summary: dict[int, str] = {}
        self.buffer = bytearray()
        self.pending_lines: list[str] = []

    def feed(self, data: bytes) -> None:
        self.raw_binary.write(data)
        self.raw_binary.flush()
        self.buffer.extend(data)
        while b"\n" in self.buffer:
            raw_line, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            self.process_line(raw_line + b"\n")

    def process_line(self, raw_line: bytes) -> None:
        line = raw_line.decode("utf-8", errors="replace")
        self.raw_text.write(line)
        self.raw_text.flush()
        if self.verbose:
            print(line, end="")

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
        state, started = self._route_session(record, firmware_session)
        if started:
            session_raw = self.session_files[state.session][1]
            session_raw.writelines(self.pending_lines)
            self.pending_lines.clear()
        self.session_files[state.session][1].write(line)
        self.session_files[state.session][1].flush()
        state.apply(record)
        self._write_global_event(record, state.session)
        envelope = self._event_envelope(record, state.session)
        self.session_files[state.session][2].write(
            json.dumps(envelope, ensure_ascii=True, sort_keys=True) + "\n"
        )
        self.session_files[state.session][2].flush()
        self._save_session(state)

        kind = record.get("type")
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
        (folder / "summary.txt").write_text(state.render_summary(), encoding="utf-8")
        (folder / "result.json").write_text(
            json.dumps(state.as_json(), indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
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
        if self.buffer:
            self.process_line(bytes(self.buffer))
            self.buffer.clear()
        for state in self.sessions.values():
            self._save_session(state)
        for _, raw, events in self.session_files.values():
            raw.close()
            events.close()
        self.raw_binary.close()
        self.raw_text.close()
        self.events.close()
        try:
            html_path, csv_path, count = generate_reports(self.root.parent)
            print(f"Spec table updated: {html_path} ({count} sessions)")
            print(f"CSV updated: {csv_path}")
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
    with serial.Serial(port=port, baudrate=baud, timeout=0.1) as device:
        while True:
            data = device.read(4096)
            if data:
                capture.feed(data)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port (auto-selected when unambiguous)")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output", type=Path, default=Path("captures"))
    parser.add_argument("--label", default="", help="optional run-name suffix")
    parser.add_argument("--verbose", action="store_true", help="echo all firmware debug lines")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--replay", type=Path, help="parse a saved UART log instead of a COM port")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list:
        for port in available_ports():
            print(f"{port.device}\t{port.description}\tVID={getattr(port, 'vid', None)!r}")
        return 0

    capture = CaptureRun(args.output, args.label, args.verbose)
    print(f"Capture directory: {capture.root.resolve()}")
    try:
        if args.replay:
            replay(args.replay, capture)
        else:
            capture_serial(choose_port(args.port), args.baud, capture)
    except KeyboardInterrupt:
        print("\nCapture stopped.")
    finally:
        capture.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
