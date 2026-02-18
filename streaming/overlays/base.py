"""Base class for all overlays."""

from abc import ABC, abstractmethod


class OverlayBase(ABC):
    """Abstract base class that all overlays inherit from."""

    def __init__(self, enabled=True):
        self.enabled = enabled

    @abstractmethod
    def render(self, frame, context=None):
        """
        Apply overlay to frame.

        Args:
            frame: OpenCV BGR frame (numpy array)
            context: Optional dict with runtime info (frame_count, timestamp, etc.)

        Returns:
            Modified frame
        """
        pass

    def update(self):
        """Optional: called each frame for dynamic overlays."""
        pass
