import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

from pd_capture import CaptureRun, SessionState, parse_record
from pd_report import generate_reports


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
    def test_parse_tolerates_debug_prefix(self):
        record = parse_record("tx@PD1,type=result,session=7,spr=pass\r\n")
        self.assertIsNotNone(record)
        self.assertEqual(record["session"], "7")
        self.assertEqual(record["spr"], "pass")

    def test_summary_rendering(self):
        state = SessionState(1)
        for line in SAMPLE.decode().splitlines():
            record = parse_record(line)
            if record:
                state.apply(record)
        summary = state.render_summary()
        self.assertIn("5 PDO / PPS none / Max 100W", summary)
        self.assertIn("28V, 36V, 48V / AVS 15V-48V / PDP 240W", summary)
        self.assertIn("Passive / 5A / 48V / USB4 Gen4", summary)
        self.assertIn("Revision : PD 3.1 / USB 1.8", summary)
        self.assertIn("Manufacturer : Example Power", summary)
        self.assertIn("EPR Exit     : Soft Reset", summary)
        self.assertIn("Overall      : PASS", summary)

    def test_capture_writes_session_artifacts(self):
        with tempfile.TemporaryDirectory() as temp:
            capture = CaptureRun(Path(temp))
            with contextlib.redirect_stdout(io.StringIO()):
                capture.feed(SAMPLE)
            root = capture.root
            capture.close()
            session = root / "session_0001"
            self.assertTrue((root / "raw.bin").exists())
            self.assertTrue((session / "events.jsonl").exists())
            self.assertIn("Overall      : PASS", (session / "summary.txt").read_text())
            self.assertIn('"overall": "PASS"', (session / "result.json").read_text())
            self.assertTrue((root.parent / "spec_table.html").exists())
            self.assertTrue((root.parent / "spec_table.csv").exists())

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
                    "summary": {
                        "pdo_count": "7",
                        "pps_count": "3",
                        "max_mw": "100000",
                        "epr_capable": "1",
                    },
                    "pdos": {},
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
            self.assertIn("Passive / 5A / 48V / USB2 only", report)
            self.assertIn("session_0001/raw.log", report)
            self.assertIn("Anker", csv_path.read_text(encoding="utf-8-sig").title())


if __name__ == "__main__":
    unittest.main()
