"""Status overlay for frame counter and recording indicator."""

import cv2

from .base import OverlayBase


class StatusOverlay(OverlayBase):
    """Displays frame counter and REC indicator."""

    def __init__(self, enabled=True, font_scale=1.0, frame_pos=(10, 30), rec_offset=100):
        super().__init__(enabled)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = font_scale
        self.thickness = 2
        self.frame_pos = frame_pos
        self.rec_offset = rec_offset  # Offset from right edge for REC indicator

        # Colors (BGR)
        self.frame_color = (0, 255, 0)  # Green
        self.rec_color = (0, 0, 255)  # Red

    def render(self, frame, context=None):
        """Render frame counter and REC indicator."""
        if context is None:
            return frame

        frame_height, frame_width = frame.shape[:2]

        # Frame counter
        frame_count = context.get("frame_count", 0)
        cv2.putText(frame, f"Frame: {frame_count}", self.frame_pos,
                    self.font, self.font_scale, self.frame_color, self.thickness)

        # REC indicator (if recording)
        if context.get("recording", False):
            rec_x = frame_width - self.rec_offset
            cv2.putText(frame, "REC", (rec_x, self.frame_pos[1]),
                        self.font, self.font_scale, self.rec_color, self.thickness)

        return frame
