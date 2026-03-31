from __future__ import annotations

import logging
from pathlib import Path

from PyQt6.QtCore import QThread, pyqtSignal
from werkzeug.serving import make_server

from backend.runtime_context import AppRuntime


class FlaskThread(QThread):
    """Run the reference Flask API inside a managed QThread."""

    server_started = pyqtSignal(str)
    server_stopped = pyqtSignal()
    error_occurred = pyqtSignal(str)

    def __init__(self, runtime: AppRuntime) -> None:
        super().__init__()
        self._runtime = runtime
        self._server = None

    def run(self) -> None:
        backend_dir = Path(__file__).resolve().parent
        project_dir = backend_dir.parent
        import sys

        if str(backend_dir) not in sys.path:
            sys.path.insert(0, str(backend_dir))
        if str(project_dir) not in sys.path:
            sys.path.insert(0, str(project_dir))

        try:
            import robot_api

            robot_api.commander = self._runtime.commander
            robot_api.shared_string = self._runtime.shared_string
            self._server = make_server("127.0.0.1", 8000, robot_api.app, threaded=True)
            self.server_started.emit("http://localhost:8000")
            self._server.serve_forever()
        except Exception as exc:
            message = f"Flask thread failed: {exc}"
            logging.exception(message)
            self.error_occurred.emit(message)
        finally:
            self._server = None
            self.server_stopped.emit()

    def stop(self) -> None:
        """Ask the Flask server to stop and wait for the thread to exit."""
        if self._server is not None:
            self._server.shutdown()
        self.wait(3000)
