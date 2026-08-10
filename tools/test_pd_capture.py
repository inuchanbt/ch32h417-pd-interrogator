import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import pd_capture
from pd_capture import (
    CaptureRun,
    SessionState,
    build_parser,
    capture_serial,
    decode_cable_vdo_from_log,
    decode_source_cap_ext_from_log,
    parse_record,
)
from pd_report import DEFAULT_CAPTURES, automatic_cable_id, decode_pdo, generate_reports


SAMPLE = b"""boot noise\r\n
@PD1,type=attach,session=1,cc=2\r\n
@PD1,type=spr,session=1,pdo_count=5,pps_count=0,max_mw=100000,epr_capable=1\r\n
@PD1,type=epr,session=1,object_count=11,complete=1,fixed_count=3,avs_min_mv=15000,avs_max_mv=48000,pdp_w=240\r\n
@PD1,type=epr_fixed,session=1,index=0,mv=28000,ma=5000\r\n
@PD1,type=epr_fixed,session=1,index=1,mv=36000,ma=5000\r\n
@PD1,type=epr_fixed,session=1,index=2,mv=48000,ma=5000\r\n
tx@PD1,type=identity,session=1,status=pass,probe=nak,vid=0000,pid=0000,id_header=01C00000,product_vdo=00000000\r\n
@PD1,type=cable,session=1,status=pass,cable_type=3,current_ma=5000,max_mv=48000,usb_speed=4,vid=0000,pid=0000,id_header=1C600000,product_vdo=00000000,cable_vdo=000A4644\r\n
@PD1,type=probe,session=1,name=revision,status=pass\r\n
@PD1,type=revision,session=1,pd_major=3,pd_minor=1,usb_major=1,usb_minor=8,raw=31180000\r\n
@PD1,type=probe,session=1,name=manufacturer_info,status=pass\r\n
@PD1,type=manufacturer,session=1,vid=29CF,pid=7209,name=Example_Power\r\n
@PD1,type=result,session=1,spr=pass,pps=na,epr=pass,chunk=pass,exit=soft_reset,source_probe=nak,complete=1,final=0\r\n
@PD1,type=detach,session=1\r\n
"""


class CaptureTests(unittest.TestCase):
    def test_spr_avs_apdo_decoding(self):
        decoded = decode_pdo(0xE404B1F4, 5, "SPR")
        self.assertEqual(decoded["kind"], "APDO_SPR_AVS")
        self.assertEqual(decoded["min_mv"], 9000)
        self.assertEqual(decoded["max_mv"], 20000)
        self.assertEqual(decoded["max_ma_15v"], 3000)
        self.assertEqual(decoded["max_ma_20v"], 5000)
        self.assertEqual(decoded["max_mw"], 100000)
        self.assertEqual(decoded["peak_current"], "1")
        self.assertIn("SPR AVS 9-20V (9-15V @ 3A / 15-20V @ 5A)", decoded["description"])

    def test_epr_avs_apdo_decoding_is_unchanged(self):
        decoded = decode_pdo(0xD7C096F0, 11, "EPR")
        self.assertEqual(decoded["kind"], "APDO_EPR_AVS")
        self.assertEqual(decoded["min_mv"], 15000)
        self.assertEqual(decoded["max_mv"], 48000)
        self.assertEqual(decoded["pdp_w"], 240)
        self.assertIn("AVS 15-48V PDP 240W", decoded["description"])

    def test_spr_avs_renders_in_session_summary(self):
        state = SessionState(1)
        state.apply({
            "type": "spr", "pdo_count": "5", "pps_count": "0",
            "spr_avs_count": "1", "max_mw": "100000", "epr_capable": "1",
        })
        state.apply({"type": "spr_pdo", "index": "5", "raw": "E404B1F4"})
        self.assertIn(
            "SPR AVS : 9-20V (9-15V @ 3A / 15-20V @ 5A)",
            state.render_summary(),
        )

    def test_default_capture_path_is_project_relative(self):
        self.assertEqual(build_parser().parse_args([]).output, DEFAULT_CAPTURES)

    def test_zero_cable_identity_has_no_automatic_id(self):
        self.assertEqual(
            automatic_cable_id({"status": "unknown", "cable_vdo": "00000000"}),
            "",
        )

    def test_parse_tolerates_debug_prefix(self):
        record = parse_record("tx@PD1,type=result,session=7,spr=pass\r\n")
        self.assertIsNotNone(record)
        self.assertEqual(record["session"], "7")
        self.assertEqual(record["spr"], "pass")

    def test_passive_cable_vdo_is_recovered_before_attach(self):
        recovered = decode_cable_vdo_from_log("  Passive Cable VDO1:0x000A2642\r\n")
        self.assertIsNotNone(recovered)
        self.assertEqual(recovered["current_ma"], "5000")
        self.assertEqual(recovered["max_mv"], "48000")
        self.assertEqual(recovered["usb_speed_name"], "usb4_gen2")

        stream = b"""Passive Cable VDO1:0x000A2642\r\n
@PD1,type=attach,session=1,cc=2\r\n
@PD1,type=cable,session=1,status=unknown,cable_vdo=00000000\r\n
"""
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(stream)
            root = capture.root
            capture.close()
            result = json.loads((root / "session_0001" / "result.json").read_text())
            self.assertEqual(result["cable"]["status"], "pass")
            self.assertEqual(result["cable"]["cable_vdo"], "000A2642")

    def test_source_cap_ext_details_render_in_summary(self):
        state = SessionState(1)
        state.apply({
            "type": "source_cap_ext", "vid": "0000", "pid": "0000",
            "xid": "00000000", "fw_version": "5", "hw_version": "1",
            "spr_pdp_w": "100", "epr_pdp_w": "240",
        })
        self.assertIn("FW 5 / HW 1 / SPR PDP 100W / EPR PDP 240W", state.render_summary())

    def test_source_cap_ext_is_recovered_from_raw_payload(self):
        line = (
            "  EXT payload bytes: [0]=00 [1]=00 [2]=00 [3]=00 [4]=00 [5]=00 "
            "[6]=00 [7]=00 [8]=05 [9]=01 [10]=00 [11]=00 [12]=00 [13]=00 "
            "[14]=00 [15]=00 [16]=00 [17]=00 [18]=00 [19]=00 [20]=00 [21]=03 "
            "[22]=00 [23]=64 [24]=F0\r\n"
        )
        recovered = decode_source_cap_ext_from_log(line)
        self.assertIsNotNone(recovered)
        self.assertEqual(recovered["fw_version"], "5")
        self.assertEqual(recovered["spr_pdp_w"], "100")
        self.assertEqual(recovered["epr_pdp_w"], "240")

        stream = (
            b"@PD1,type=attach,session=1,cc=2\r\n"
            b"@PD1,type=probe,session=1,name=source_cap_ext,status=pass\r\n"
            + line.encode()
        )
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(stream)
            root = capture.root
            capture.close()
            result = json.loads((root / "session_0001" / "result.json").read_text())
            detail = result["probes"]["details"]["source_cap_ext"]
            self.assertEqual(detail["spr_pdp_w"], "100")
            self.assertEqual(detail["epr_pdp_w"], "240")

    def test_summary_rendering(self):
        state = SessionState(1)
        for line in SAMPLE.decode().splitlines():
            record = parse_record(line)
            if record:
                state.apply(record)
        summary = state.render_summary()
        self.assertIn("5 PDO / PPS none / Max 100W", summary)
        self.assertIn("EPR Fixed : 28V, 36V, 48V", summary)
        self.assertIn("EPR AVS : 15-48V (PDP 240W)", summary)
        self.assertIn("Passive / 5A / 48V / USB4 Gen4", summary)
        self.assertIn("Revision : PD 3.1 / USB 1.8", summary)
        self.assertIn("Manufacturer : Example Power", summary)
        self.assertIn("EPR Exit     : Soft Reset", summary)
        self.assertIn("Overall      : PASS", summary)

    def test_capture_writes_session_artifacts(self):
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(
                Path(temp), source_id="test-source", cable_id="test-cable",
                cable_attachment="captive",
            )
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(SAMPLE)
            root = capture.root
            capture.close()
            session = root / "session_0001"
            self.assertTrue((root / "raw.bin").exists())
            self.assertTrue((session / "events.jsonl").exists())
            self.assertIn("Overall      : PASS", (session / "summary.txt").read_text())
            self.assertIn('"overall": "PASS"', (session / "result.json").read_text())
            self.assertIn('"source_id": "test-source"', (session / "result.json").read_text())
            self.assertIn('"cable_id": "test-cable"', (session / "result.json").read_text())
            self.assertIn('"cable_attachment": "captive"', (session / "result.json").read_text())
            self.assertTrue((root / "capture.json").exists())
            self.assertTrue((root.parent / "spec_table.html").exists())
            self.assertTrue((root.parent / "spec_table.csv").exists())
            self.assertTrue((root.parent / "source_table.csv").exists())
            self.assertTrue((root.parent / "cable_table.csv").exists())
            source_csv = (root.parent / "source_table.csv").read_text(
                encoding="utf-8-sig"
            )
            cable_csv = (root.parent / "cable_table.csv").read_text(
                encoding="utf-8-sig"
            )
            self.assertIn("captive_cable_id", source_csv)
            self.assertIn("test-cable", source_csv)
            self.assertIn("owner_source_id", cable_csv)
            self.assertIn("test-source", cable_csv)
            self.assertIn("captive", cable_csv)

    def test_cr_only_records_are_not_merged_at_detach(self):
        stream = (
            b"@PD1,type=attach,session=1,cc=2\r"
            b"@PD1,type=result,session=1,spr=pass,pps=na,epr=pass,"
            b"chunk=pass,exit=source_cap,source_probe=pass,complete=1,final=1\r"
            b"@PD1,type=probe,session=1,name=battery_status,status=no_response\r"
            b"@PD1,type=detach,session=1\r"
        )
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(stream)
            root = capture.root
            capture.close()
            result = json.loads((root / "session_0001" / "result.json").read_text())
            self.assertEqual(result["protocol"]["final"], "1")
            self.assertEqual(
                result["probes"]["status"]["battery_status"], "no_response"
            )
            self.assertTrue(result["detached"])
            events = [
                json.loads(line)
                for line in (root / "events.jsonl").read_text().splitlines()
            ]
            self.assertEqual(
                [event["type"] for event in events],
                ["attach", "result", "probe", "detach"],
            )

    def test_serial_disconnect_reconnects_without_ending_capture(self):
        class FakeSerialException(Exception):
            pass

        class FakeDevice:
            def __init__(self, reads):
                self.reads = iter(reads)

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return False

            def read(self, _size):
                item = next(self.reads)
                if isinstance(item, BaseException):
                    raise item
                return item

        devices = iter(
            [
                FakeDevice([FakeSerialException("device vanished")]),
                FakeDevice([b"reconnected\r\n", KeyboardInterrupt()]),
            ]
        )
        fake_serial = mock.Mock()
        fake_serial.SerialException = FakeSerialException
        fake_serial.Serial.side_effect = lambda **_kwargs: next(devices)
        capture = mock.Mock()

        with (
            mock.patch.object(pd_capture, "serial", fake_serial),
            mock.patch.object(pd_capture.time, "sleep"),
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(io.StringIO()),
            self.assertRaises(KeyboardInterrupt),
        ):
            capture_serial("COM7", 921600, capture)

        capture.feed.assert_called_once_with(b"reconnected\r\n")
        self.assertEqual(fake_serial.Serial.call_count, 2)

    def test_report_interrupt_after_ctrl_c_keeps_capture_files_saved(self):
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(SAMPLE)
            root = capture.root
            stderr = io.StringIO()
            with (
                mock.patch.object(
                    pd_capture, "generate_reports", side_effect=KeyboardInterrupt
                ),
                contextlib.redirect_stderr(stderr),
            ):
                capture.close()

            self.assertTrue(capture.closed)
            self.assertTrue(capture.raw_binary.closed)
            self.assertTrue(capture.raw_text.closed)
            self.assertTrue(capture.events.closed)
            self.assertTrue((root / "session_0001" / "result.json").exists())
            self.assertIn("capture files were saved successfully", stderr.getvalue())
            capture.close()

    def test_atomic_write_retries_transient_windows_permission_error(self):
        with tempfile.TemporaryDirectory() as temp:
            target = Path(temp) / "summary.txt"
            target.write_text("old", encoding="utf-8")
            real_replace = os.replace
            attempts = 0

            def flaky_replace(source, destination):
                nonlocal attempts
                attempts += 1
                if attempts < 3:
                    raise PermissionError(13, "temporarily locked", destination)
                return real_replace(source, destination)

            with (
                mock.patch.object(pd_capture.os, "replace", side_effect=flaky_replace),
                mock.patch.object(pd_capture.time, "sleep"),
            ):
                pd_capture.atomic_write_text(target, "new")

            self.assertEqual(attempts, 3)
            self.assertEqual(target.read_text(encoding="utf-8"), "new")

    def test_persistent_session_file_lock_does_not_crash_close(self):
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(SAMPLE)
            root = capture.root
            stderr = io.StringIO()
            with (
                mock.patch.object(
                    pd_capture,
                    "atomic_write_text",
                    side_effect=PermissionError(13, "locked"),
                ),
                contextlib.redirect_stderr(stderr),
            ):
                capture.close()

            self.assertTrue(capture.closed)
            self.assertTrue((root / "session_0001" / "summary.txt").exists())
            self.assertTrue((root / "session_0001" / "result.json").exists())
            self.assertIn("keeping the previous saved version", stderr.getvalue())

    def test_mcu_boot_starts_monotonic_host_session(self):
        stream = b"""@PD1,type=boot,session=0\r\n
@PD1,type=attach,session=2,cc=1\r\n
@PD1,type=result,session=2,spr=pass,pps=na,epr=na,chunk=na,exit=unknown,source_probe=pass,complete=1,final=0\r\n
@PD1,type=boot,session=0\r\n
@PD1,type=attach,session=1,cc=1\r\n
@PD1,type=result,session=1,spr=pass,pps=na,epr=na,chunk=na,exit=unknown,source_probe=pass,complete=1,final=0\r\n
"""
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(stream)
            root = capture.root
            capture.close()

            first = json.loads((root / "session_0001" / "result.json").read_text())
            second = json.loads((root / "session_0002" / "result.json").read_text())
            self.assertEqual((first["session"], first["firmware_session"]), (1, 2))
            self.assertEqual((second["session"], second["firmware_session"]), (2, 1))

    def test_duplicate_result_summary_is_suppressed(self):
        result = b"""@PD1,type=attach,session=1,cc=1\r\n
@PD1,type=spr,session=1,pdo_count=1,pps_count=0,max_mw=15000,epr_capable=0\r\n
@PD1,type=result,session=1,spr=pass,pps=na,epr=na,chunk=na,exit=unknown,source_probe=pass,complete=1,final=0\r\n
@PD1,type=result,session=1,spr=pass,pps=na,epr=na,chunk=na,exit=unknown,source_probe=pass,complete=1,final=1\r\n
"""
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                capture.feed(result)
            capture.close()
            self.assertEqual(output.getvalue().count("[Source Summary]"), 1)

    def test_epr_completion_is_shown_before_identity_finishes(self):
        stream = b"""@PD1,type=attach,session=1,cc=1\r\n
@PD1,type=spr,session=1,pdo_count=7,pps_count=2,max_mw=105000,epr_capable=1\r\n
@PD1,type=epr,session=1,object_count=9,complete=1,fixed_count=1,avs_min_mv=15000,avs_max_mv=28000,pdp_w=140\r\n
@PD1,type=epr_pdo,session=1,index=8,raw=0088C1F4\r\n
@PD1,type=result,session=1,spr=pass,pps=pass,epr=pass,chunk=pass,exit=unknown,source_probe=unknown,complete=0,final=0\r\n
"""
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                capture.feed(stream)
            capture.close()
            self.assertIn("protocol complete; identity pending", output.getvalue())
            self.assertIn("Overall      : PASS", output.getvalue())

    def test_corrupted_pdo_record_is_ignored(self):
        state = SessionState(1)
        state.apply({"type": "epr_pdo", "index": "8", "raw": "0088C=identity"})
        state.apply({"type": "epr_pdo", "index": "9", "raw": "d230968c"})
        self.assertNotIn(8, state.epr_pdos)
        self.assertEqual(state.epr_pdos[9], "D230968C")

    def test_epr_capability_timeout_is_ready_as_partial(self):
        state = SessionState(1)
        state.apply({"type": "spr", "epr_capable": "1"})
        state.apply(
            {
                "type": "result",
                "spr": "pass",
                "pps": "na",
                "epr": "pass",
                "chunk": "no_response",
                "complete": "0",
            }
        )
        self.assertTrue(state.protocol_measurement_ready())
        self.assertEqual(state.overall(), "PARTIAL")

    def test_report_contains_capabilities_and_links(self):
        with tempfile.TemporaryDirectory() as temp:
            captures = Path(temp)
            session = captures / "20260713_130102_anker" / "session_0001"
            session.mkdir(parents=True)
            result = {
                "session": 1,
                "firmware_session": 2,
                "source": {
                    "pdos": {
                        "1": "2A81912C",
                        "2": "0002D12C",
                        "7": "C8DC213C",
                    },
                    "summary": {
                        "pdo_count": "7",
                        "pps_count": "3",
                        "max_mw": "100000",
                        "epr_capable": "1",
                    },
                },
                "epr": {
                    "summary": {
                        "complete": "1",
                        "avs_min_mv": "15000",
                        "avs_max_mv": "28000",
                        "pdp_w": "140",
                    },
                    "fixed": {"0": [28000, 5000]},
                    "pdos": {},
                },
                "identity": {"status": "no_response"},
                "probes": {
                    "status": {
                        "revision": "pass",
                        "source_info": "pass",
                        "country_codes": "unsupported",
                        "manufacturer_info": "pass",
                    },
                    "details": {
                        "revision": {
                            "pd_major": "3",
                            "pd_minor": "1",
                            "usb_major": "1",
                            "usb_minor": "8",
                        },
                        "source_info": {
                            "pdp_w": "140",
                            "temp_raw": "255",
                            "present_input": "1",
                        },
                        "manufacturer": {
                            "name": "Example_Power",
                            "vid": "29CF",
                            "pid": "7209",
                        },
                    },
                },
                "cable": {
                    "status": "pass",
                    "cable_type": "3",
                    "current_ma": "5000",
                    "max_mv": "48000",
                    "usb_speed": "0",
                    "vid": "2C19",
                },
                "protocol": {
                    "spr": "pass",
                    "pps": "pass",
                    "epr": "pass",
                    "chunk": "pass",
                    "exit": "soft_reset",
                },
                "overall": "PASS",
                "detached": True,
            }
            (session / "result.json").write_text(json.dumps(result), encoding="utf-8")
            (session / "summary.txt").write_text("summary", encoding="utf-8")
            (session / "raw.log").write_text("raw", encoding="utf-8")

            html_path, csv_path, count = generate_reports(captures)
            report = html_path.read_text(encoding="utf-8")
            self.assertEqual(count, 1)
            self.assertIn("7 PDO / Max 100W", report)
            self.assertIn("15V-28V", report)
            self.assertIn("PD 3.1 / USB 1.8", report)
            self.assertIn("PDP 140W / Temp raw 255 / Input AC", report)
            self.assertIn("CountryCodes=UNSUPPORTED", report)
            self.assertIn("source_info: pdp_w=140", report)
            self.assertIn("Power: DRP / Data: DRD", report)
            self.assertIn("#1 Fixed 5V @ 3A (0x2A81912C)", report)
            self.assertIn("Passive / 5A / 48V / USB2 only", report)
            self.assertIn("session_0001/raw.log", report)
            self.assertIn('href="pdo_table.csv"', report)
            self.assertIn('href="source_table.csv"', report)
            self.assertIn('href="cable_table.csv"', report)
            self.assertIn("Anker", csv_path.read_text(encoding="utf-8-sig").title())
            pdo_csv = (captures / "pdo_table.csv").read_text(encoding="utf-8-sig")
            self.assertIn("dual_role_power", pdo_csv)
            self.assertIn("0x2A81912C", pdo_csv)
            self.assertIn("APDO_SPR_PPS", pdo_csv)
            self.assertIn("source_id", pdo_csv)
            self.assertNotIn("source_name", pdo_csv)
            cable_csv = (captures / "cable_table.csv").read_text(encoding="utf-8-sig")
            self.assertIn("cable_id", cable_csv)
            self.assertIn("auto-emarker-", cable_csv)

    def test_captive_cable_derives_id_and_ownership(self):
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(
                Path(temp), source_id="phihong", cable_attachment="captive"
            )
            self.assertEqual(capture.cable_id, "phihong:captive")
            capture.close()

    def test_capture_folder_uses_source_manufacturer_and_model(self):
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(
                Path(temp),
                source_id="stable-source-id",
                source_manufacturer="Example Corp",
                source_model="A/B 240W",
            )
            self.assertRegex(
                capture.root.name,
                r"^\d{8}_\d{6}_Example-Corp_A-B-240W$",
            )
            self.assertNotIn("stable-source-id", capture.root.name)
            capture.close()


if __name__ == "__main__":
    unittest.main()
