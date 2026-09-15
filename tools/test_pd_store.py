import contextlib
import csv
import io
import json
import sqlite3
import tempfile
import unittest
from pathlib import Path
from unittest import mock
from urllib.parse import unquote

import pd_capture
import pd_report
import pd_store
from test_pd_capture import SAMPLE


class StorageTests(unittest.TestCase):
    def test_asd_filename_tokens_and_length(self):
        self.assertEqual(pd_store.filename_token(' Example / Cable 1.8m '), 'Example-Cable-1p8m')
        stem = pd_store.measurement_stem(dict(source_name='AOHI AOC-C022', cable_name='Cable 1.8m',
                                             measurement_condition='pd-interrogate'), '20260914_153000')
        self.assertEqual(stem, 'AOHI-AOC-C022__Cable-1p8m__pd-interrogate__20260914_153000')
        long = pd_store.measurement_stem(dict(source_name='A' * 100, cable_name='B' * 100,
                                            measurement_condition='C' * 100), '20260914_153000', 70)
        self.assertEqual(len(long), 70)
        self.assertEqual(len(long.split('__')), 4)

    def test_named_logs_preserve_bytes_metadata_links_and_index(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()):
            output = Path(temp)
            run = pd_capture.CaptureRun(output, source_id='adapter-1', cable_id='cable-1',
                                       source_name='AOHI AOC-C022', firmware_version='r70',
                                       board_revision='rev2', cc_resistance='5.1k', orientation='ura')
            run.feed(SAMPLE)
            run.close()
            self.assertEqual(run.raw_binary_path.read_bytes(), SAMPLE)
            self.assertIn('__pd-interrogate__', run.raw_log_path.name)
            metadata = json.loads((run.root / 'capture.json').read_text())
            self.assertEqual(metadata['artifacts']['raw_log'], run.raw_log_path.name)
            result_path = run.root / 'session_0001' / 'result.json'
            result = json.loads(result_path.read_text())
            self.assertTrue((result_path.parent / result['artifacts']['raw_log']).is_file())
            self.assertEqual(result['firmware_version'], 'r70')
            rows = pd_store.history(output, source_id='adapter-1')
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['Firmware Version'], 'r70')
            self.assertEqual(rows[0]['Orientation'], 'ura')
            self.assertTrue((output / unquote(rows[0]['_raw'])).is_file())
            with (output / 'spec_table.csv').open(encoding='utf-8-sig', newline='') as stream:
                exported = list(csv.DictReader(stream))
            self.assertEqual(exported[0]['Result ID'], result['result_id'])
            self.assertEqual(exported[0]['Raw Log'], rows[0]['Raw Log'])
            self.assertNotIn(b'\r\r\n', (output / 'spec_table.csv').read_bytes())
            pd_store.save_profile(output, 'bench', {'source_id': 'adapter-1'})
            pd_report.save_favorite(output, result['result_id'], True)
            pd_report.generate_reports(output)
            self.assertEqual(pd_store.history(output)[0]['Favorite'], 'Yes')
            self.assertEqual(pd_store.load_profile(output, 'bench')['source_id'], 'adapter-1')
            self.assertEqual(result_path.read_text(), json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True) + '\n')

    def test_profiles_never_connect_and_cli_overrides_are_not_saved(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()), mock.patch.object(pd_capture, 'capture_serial') as serial:
            output = Path(temp)
            pd_capture.main(['--output', temp, '--save-profile', 'bench', '--source-id', 'one', '--firmware-version', 'old'])
            serial.assert_not_called()
            self.assertFalse(list(output.glob('*/capture.json')))
            log = output / 'input.log'
            log.write_bytes(SAMPLE)
            pd_capture.main(['--output', temp, '--profile', 'bench', '--firmware-version', 'new', '--replay', str(log), '--quiet'])
            self.assertEqual(pd_store.load_profile(output, 'bench')['firmware_version'], 'old')
            self.assertEqual(pd_store.history(output)[0]['Firmware Version'], 'new')
            self.assertEqual(pd_store.history(output)[0]['Source ID'], 'one')

    def test_unknown_profile_and_invalid_input_do_not_create_capture(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                pd_capture.main(['--output', temp, '--profile', 'missing'])
            with self.assertRaises(SystemExit):
                pd_capture.main(['--output', temp, '--replay', str(Path(temp) / 'missing.log')])
            with self.assertRaises(SystemExit):
                pd_capture.main(['--output', temp, '--input-ac-voltage', 'nan', '--save-profile', 'bad'])
            self.assertFalse(list(Path(temp).glob('*/capture.json')))

    def test_register_existing_capture_and_override_its_metadata(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()):
            output = Path(temp)
            original = output / 'original'
            original.mkdir()
            (original / 'capture.json').write_text(json.dumps({'format': 'PD_CAPTURE_1',
                'source_id': 'original', 'firmware_version': 'old', 'capture_id': 'do-not-copy'}))
            pd_capture.main(['--output', temp, '--from-capture', str(original),
                             '--firmware-version', 'new', '--save-profile', 'imported'])
            profile = pd_store.load_profile(output, 'imported')
            self.assertEqual(profile['source_id'], 'original')
            self.assertEqual(profile['firmware_version'], 'new')
            self.assertNotIn('capture_id', profile)

    def test_reusing_interactive_setup_does_not_require_retyping(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()):
            pd_capture.main(['--output', temp, '--save-profile', 'bench', '--source-id', 'one'])
            with mock.patch('builtins.input', side_effect=['', '', '', 'n']) as prompts:
                self.assertEqual(pd_capture.main(['--output', temp, '--setup']), 0)
            self.assertEqual(prompts.call_count, 4)
            self.assertFalse(list(Path(temp).glob('*/capture.json')))

    def test_atomic_csv_failure_keeps_previous_export(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'spec_table.csv'
            path.write_text('previous', encoding='utf-8')
            with mock.patch.object(pd_report.os, 'replace', side_effect=PermissionError('locked')):
                with self.assertRaises(PermissionError):
                    pd_report.write_csv(path, [])
            self.assertEqual(path.read_text(), 'previous')
            self.assertEqual(list(Path(temp).glob('*.tmp')), [])

    def test_index_rebuild_rolls_back_on_duplicate_ids(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            row = {'Result ID': 'id-1', 'Captured': '2026-01-01', 'Source ID': 's', 'Cable ID': 'c',
                   'Result Status': 'Valid', 'Favorite': 'No', '_result': 'a/result.json'}
            pd_store.index_rows(path, [row])
            with self.assertRaises(sqlite3.IntegrityError):
                pd_store.index_rows(path, [row, row])
            self.assertEqual(len(pd_store.history(path)), 1)

    def test_capture_directory_collision_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()):
            with mock.patch.object(pd_capture, 'datetime') as clock:
                import datetime
                clock.now.return_value = datetime.datetime(2026, 9, 14, 12)
                first = pd_capture.CaptureRun(Path(temp))
                second = pd_capture.CaptureRun(Path(temp))
            first.feed(b'first\n')
            second.feed(b'second\n')
            first.close()
            second.close()
            self.assertNotEqual(first.root, second.root)
            self.assertTrue(second.root.name.endswith('_02'))
            self.assertTrue(second.raw_binary_path.stem.endswith('_02'))
            self.assertEqual(first.raw_binary_path.read_bytes(), b'first\n')
            self.assertEqual(second.raw_binary_path.read_bytes(), b'second\n')
