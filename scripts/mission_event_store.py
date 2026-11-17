"""Durable idempotent event journal for mission lifecycle and deliveries."""
from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


class MissionEventStore:
    """Append-only SQLite journal with event IDs as idempotency keys."""

    def __init__(self, database_path: str | Path) -> None:
        self.database_path = str(database_path)
        if self.database_path != ":memory:":
            path = Path(self.database_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.database_path = str(path)
        self.connection = sqlite3.connect(self.database_path, timeout=10.0)
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=FULL")
        self.connection.execute(
            "CREATE TABLE IF NOT EXISTS mission_events ("
            "event_id TEXT PRIMARY KEY, event_type TEXT NOT NULL, "
            "goal_id TEXT NOT NULL, payload_json TEXT NOT NULL, "
            "created_utc TEXT NOT NULL)"
        )
        self.connection.commit()

    def record(self, event_id: str, event_type: str, goal_id: str = "",
               payload: dict[str, Any] | None = None) -> dict[str, Any]:
        if not event_id or not event_type or "\x00" in event_id:
            raise ValueError("event ID and type must be non-empty text")
        payload_json = json.dumps(payload or {}, sort_keys=True,
                                  separators=(",", ":"), allow_nan=False)
        created = datetime.now(timezone.utc).isoformat()
        with self.connection:
            cursor = self.connection.execute(
                "INSERT OR IGNORE INTO mission_events "
                "(event_id,event_type,goal_id,payload_json,created_utc) "
                "VALUES (?,?,?,?,?)",
                (event_id, event_type, goal_id, payload_json, created))
            inserted = cursor.rowcount == 1
        row = self.connection.execute(
            "SELECT event_id,event_type,goal_id,payload_json,created_utc "
            "FROM mission_events WHERE event_id=?", (event_id,)).fetchone()
        if row is None:
            raise RuntimeError("event insert/read did not return a row")
        return {
            "inserted": inserted,
            "event_id": row[0],
            "event_type": row[1],
            "goal_id": row[2],
            "payload": json.loads(row[3]),
            "created_utc": row[4],
        }

    def get(self, event_id: str) -> dict[str, Any] | None:
        row = self.connection.execute(
            "SELECT event_id,event_type,goal_id,payload_json,created_utc "
            "FROM mission_events WHERE event_id=?", (event_id,)).fetchone()
        if row is None:
            return None
        return {
            "event_id": row[0], "event_type": row[1], "goal_id": row[2],
            "payload": json.loads(row[3]), "created_utc": row[4],
        }

    def count(self, event_type: str | None = None) -> int:
        if event_type is None:
            row = self.connection.execute(
                "SELECT COUNT(*) FROM mission_events").fetchone()
        else:
            row = self.connection.execute(
                "SELECT COUNT(*) FROM mission_events WHERE event_type=?",
                (event_type,)).fetchone()
        return int(row[0])

    def close(self) -> None:
        self.connection.close()

    def __enter__(self) -> "MissionEventStore":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
