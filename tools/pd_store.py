"""Local settings registry and rebuildable measurement index (standard library only)."""
from __future__ import annotations

import json
import re
import sqlite3
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from typing import Iterator


DATABASE_NAME = "measurements.sqlite3"
PROFILE_FIELDS = (
    "source_id", "source_name", "source_manufacturer", "source_model", "source_port",
    "cable_id", "cable_name", "cable_manufacturer", "cable_model", "cable_length_m",
    "cable_attachment", "input_ac_voltage_v", "input_ac_frequency_hz",
    "firmware_version", "board_revision", "cc_resistance", "orientation",
    "measurement_condition", "test_note",
)


def filename_token(value: object, fallback: str = "") -> str:
    """Match ASD-PD31's Windows-safe filename component rules."""
    text = str(value or "").strip()
    text = re.sub(r'[<>:"/\\|?*\x00-\x1f]+', "_", text)
    text = re.sub(r"\s+", "-", text)
    text = re.sub(r"[-_]{2,}", "-", text).strip(" .-_")
    return text.replace(".", "p") or fallback


def measurement_stem(metadata: dict[str, object], stamp: str, max_length: int = 209) -> str:
    source = metadata.get("source_name") or "-".join(
        str(metadata.get(key) or "") for key in ("source_manufacturer", "source_model", "source_port")
        if metadata.get(key)
    ) or metadata.get("source_id")
    # An unidentified source should use its stable ID before the default port.
    if not metadata.get("source_name") and not any(metadata.get(k) for k in ("source_manufacturer", "source_model")):
        source = metadata.get("source_id") or "source"
    cable = metadata.get("cable_name") or "-".join(
        str(metadata.get(key) or "") for key in ("cable_manufacturer", "cable_model")
        if metadata.get(key)
    ) or metadata.get("cable_id") or "cable"
    if not metadata.get("cable_name"):
        if metadata.get("cable_length_m") is not None:
            cable = f"{cable}-{float(metadata['cable_length_m']):g}m"
        if metadata.get("cable_attachment") == "captive":
            cable = f"{cable}-captive"
    condition = metadata.get("measurement_condition") or "pd-interrogate"
    parts = [filename_token(source, 'source')[:48], filename_token(cable, 'cable')[:48], filename_token(condition, 'pd-interrogate')[:92]]
    budget = max_length - len(stamp) - 6
    if budget < 12:
        raise ValueError("Capture path is too long; choose a shorter --output directory")
    while sum(map(len, parts)) > budget:
        longest = max(range(3), key=lambda i: len(parts[i]))
        parts[longest] = parts[longest][:-1]
    return "__".join([*parts, stamp])


@contextmanager
def database(captures: Path) -> Iterator[sqlite3.Connection]:
    captures.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(captures / DATABASE_NAME, timeout=5)
    connection.row_factory = sqlite3.Row
    try:
        connection.executescript("""
            CREATE TABLE IF NOT EXISTS profiles (
                name TEXT PRIMARY KEY, metadata_json TEXT NOT NULL, updated_at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS settings (key TEXT PRIMARY KEY, value TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS sessions (
                result_id TEXT PRIMARY KEY, captured TEXT NOT NULL,
                source_id TEXT NOT NULL, cable_id TEXT NOT NULL,
                status TEXT NOT NULL, favorite INTEGER NOT NULL,
                result_path TEXT NOT NULL, row_json TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS sessions_source_date ON sessions(source_id, captured);
            CREATE INDEX IF NOT EXISTS sessions_cable_date ON sessions(cable_id, captured);
        """)
        with connection:
            yield connection
    finally:
        connection.close()


def list_profiles(captures: Path) -> list[dict[str, object]]:
    with database(captures) as connection:
        return [dict(name=row['name'], metadata=json.loads(row['metadata_json']))
                for row in connection.execute("SELECT * FROM profiles ORDER BY name COLLATE NOCASE")]


def load_profile(captures: Path, name: str) -> dict[str, object]:
    with database(captures) as connection:
        row = connection.execute("SELECT metadata_json FROM profiles WHERE name=?", (name,)).fetchone()
    if row is None:
        raise ValueError(f"Unknown profile: {name}. Use --profiles to list saved settings.")
    return json.loads(row[0])


def save_profile(captures: Path, name: str, metadata: dict[str, object]) -> None:
    name = name.strip()
    if not name or len(name) > 120:
        raise ValueError("Profile name must contain 1-120 characters")
    values = {key: metadata[key] for key in PROFILE_FIELDS if key in metadata}
    with database(captures) as connection:
        connection.execute("INSERT INTO profiles VALUES (?, ?, ?) ON CONFLICT(name) DO UPDATE SET metadata_json=excluded.metadata_json, updated_at=excluded.updated_at",
                           (name, json.dumps(values, ensure_ascii=False), datetime.now().astimezone().isoformat(timespec='seconds')))
        connection.execute("INSERT INTO settings VALUES ('last_profile', ?) ON CONFLICT(key) DO UPDATE SET value=excluded.value", (name,))


def last_profile(captures: Path) -> str:
    with database(captures) as connection:
        row = connection.execute("SELECT value FROM settings WHERE key='last_profile'").fetchone()
    return row[0] if row else ""


def index_rows(captures: Path, rows: list[dict[str, str]]) -> None:
    """Replace only the derived index, atomically; never erase saved profiles."""
    with database(captures) as connection:
        connection.execute("DELETE FROM sessions")
        connection.executemany("INSERT INTO sessions VALUES (?, ?, ?, ?, ?, ?, ?, ?)", [
            (row['Result ID'], row['Captured'], row['Source ID'], row['Cable ID'],
             row['Result Status'], int(row['Favorite'] == 'Yes'), row['_result'],
             json.dumps(row, ensure_ascii=False)) for row in rows
        ])


def history(captures: Path, source_id: str = "", cable_id: str = "", limit: int = 50) -> list[dict[str, str]]:
    with database(captures) as connection:
        rows = connection.execute(
            "SELECT row_json FROM sessions WHERE (?='' OR source_id=?) AND (?='' OR cable_id=?) ORDER BY captured DESC, result_id LIMIT ?",
            (source_id, source_id, cable_id, cable_id, max(1, min(limit, 10000))),
        ).fetchall()
    return [json.loads(row[0]) for row in rows]
