class PAROL6Error(Exception):
    """Base exception for the PAROL6 control application."""


class WorkspaceViolationError(PAROL6Error):
    """Raised when a computed pick point is outside the configured workspace."""

    def __init__(self, x: float, y: float, bounds: dict) -> None:
        self.x = x
        self.y = y
        self.bounds = bounds
        super().__init__(
            f"Object at ({x:.1f}, {y:.1f}) mm is outside workspace bounds"
        )


class IKFailureError(PAROL6Error):
    """Raised when inverse kinematics cannot produce a valid target pose."""


class SerialTimeoutError(PAROL6Error):
    """Raised when the controller does not acknowledge serial commands in time."""


class FailsafeTriggeredError(PAROL6Error):
    """Raised when execution is blocked by an active failsafe state."""


class ModelNotFoundError(PAROL6Error):
    """Raised when a required ONNX model file is not found."""
