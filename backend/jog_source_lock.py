"""Mutual-exclusion lock that ensures only one jog source is active at a time.

Sources: ``'gui'`` or ``'modbus'``.  The lock auto-releases after
*timeout_s* seconds of inactivity (no ``renew()`` calls), preventing
deadlocks if a Modbus connection drops mid-jog.
"""

from __future__ import annotations

import threading
import time


class JogSourceLock:
    """Thread-safe jog source mutex with automatic timeout release."""

    def __init__(self, timeout_s: float = 5.0) -> None:
        self._timeout_s = max(0.5, timeout_s)
        self._lock = threading.Lock()
        self._owner: str | None = None
        self._last_renew: float = 0.0

    # ------------------------------------------------------------------
    @property
    def active_source(self) -> str | None:
        """Return the current owner, or *None* if the lock is free."""
        with self._lock:
            self._auto_expire()
            return self._owner

    # ------------------------------------------------------------------
    def acquire(self, source: str) -> bool:
        """Try to acquire for *source*.  Returns ``True`` on success."""
        with self._lock:
            self._auto_expire()
            if self._owner is None or self._owner == source:
                self._owner = source
                self._last_renew = time.monotonic()
                return True
            return False

    def release(self, source: str) -> None:
        """Release the lock if held by *source*."""
        with self._lock:
            if self._owner == source:
                self._owner = None

    def renew(self, source: str) -> None:
        """Extend the timeout if *source* currently holds the lock."""
        with self._lock:
            if self._owner == source:
                self._last_renew = time.monotonic()

    # ------------------------------------------------------------------
    def _auto_expire(self) -> None:
        """Release the lock if the timeout has elapsed (caller holds ``_lock``)."""
        if self._owner is not None:
            if (time.monotonic() - self._last_renew) > self._timeout_s:
                self._owner = None
