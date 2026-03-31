from __future__ import annotations

from csv import DictWriter
from datetime import datetime
from pathlib import Path
from threading import RLock
from typing import Any

import pandas as pd


class ResearchLogger:
    """Singleton CSV logger for research metrics and experiment events."""

    _instance: "ResearchLogger | None" = None
    _instance_lock = RLock()

    def __init__(self, logs_dir: Path | None = None) -> None:
        self._lock = RLock()
        self._logs_dir = logs_dir or Path(__file__).resolve().parents[1] / "logs"
        self._logs_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_path = self._logs_dir / f"research_{timestamp}.csv"
        self._fieldnames = ["timestamp", "metric", "value", "note"]
        with self._session_path.open("w", encoding="utf-8", newline="") as handle:
            writer = DictWriter(handle, fieldnames=self._fieldnames)
            writer.writeheader()

    @classmethod
    def instance(cls) -> "ResearchLogger":
        """Return the shared ResearchLogger instance."""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
        return cls._instance

    @property
    def session_path(self) -> Path:
        """Return the active session CSV path."""
        return self._session_path

    def record(self, metric_name: str, value: int | float | str, note: str = "") -> None:
        """Append one metric or event entry to the current session log."""
        row = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "metric": metric_name,
            "value": value,
            "note": note,
        }
        with self._lock:
            with self._session_path.open("a", encoding="utf-8", newline="") as handle:
                writer = DictWriter(handle, fieldnames=self._fieldnames)
                writer.writerow(row)

    def export_csv(self, destination: str | Path | None = None) -> Path:
        """Export the session CSV to a destination path and return that path."""
        with self._lock:
            destination_path = Path(destination) if destination else self._session_path
            if destination_path != self._session_path:
                dataframe = pd.read_csv(self._session_path)
                dataframe.to_csv(destination_path, index=False)
            return destination_path

    def read_records(self) -> list[dict[str, Any]]:
        """Return all recorded rows for the active session."""
        with self._lock:
            dataframe = pd.read_csv(self._session_path)
        return dataframe.to_dict(orient="records")

    def latest_value(self, metric_name: str) -> Any:
        """Return the most recent value recorded for a given metric name."""
        records = self.read_records()
        for row in reversed(records):
            if row["metric"] == metric_name:
                return row["value"]
        return None
