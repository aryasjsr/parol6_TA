from __future__ import annotations

import json
from pathlib import Path
from threading import RLock
from typing import Any


class ConfigManager:
    """Singleton config manager with dot-notation access to config.json."""

    _instance: "ConfigManager | None" = None
    _instance_lock = RLock()

    def __init__(self, config_path: Path | None = None) -> None:
        self._config_path = config_path or Path(__file__).resolve().parents[1] / "config.json"
        self._lock = RLock()
        self._config: dict[str, Any] = {}
        self.reload()

    @classmethod
    def instance(cls) -> "ConfigManager":
        """Return the shared ConfigManager instance."""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
        return cls._instance

    @property
    def config_path(self) -> Path:
        """Return the resolved path to the backing config.json file."""
        return self._config_path

    def reload(self) -> dict[str, Any]:
        """Reload config.json from disk and return a copy of the data."""
        with self._lock:
            if not self._config_path.exists():
                self._config = {}
                self._save_locked()
            else:
                self._config = json.loads(self._config_path.read_text(encoding="utf-8"))
            return self.as_dict()

    def get(self, key: str, default: Any = None) -> Any:
        """Get a top-level or dot-notation config value."""
        with self._lock:
            if not key:
                return self.as_dict()

            value: Any = self._config
            for part in key.split("."):
                if not isinstance(value, dict) or part not in value:
                    return default
                value = value[part]
            return value

    def set(self, key: str, value: Any) -> None:
        """Set a top-level or dot-notation config value and persist immediately."""
        if not key:
            raise ValueError("Config key must not be empty")

        with self._lock:
            target = self._config
            parts = key.split(".")
            for part in parts[:-1]:
                next_value = target.get(part)
                if not isinstance(next_value, dict):
                    next_value = {}
                    target[part] = next_value
                target = next_value
            target[parts[-1]] = value
            self._save_locked()

    def as_dict(self) -> dict[str, Any]:
        """Return a deep copy of the current config dictionary."""
        with self._lock:
            return json.loads(json.dumps(self._config))

    def _save_locked(self) -> None:
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        self._config_path.write_text(
            json.dumps(self._config, indent=2) + "\n",
            encoding="utf-8",
        )
