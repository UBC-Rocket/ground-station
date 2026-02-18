"""Telemetry overlay for displaying live rocket data."""

import json
import os
import threading

import cv2

from .base import OverlayBase


class TelemetryOverlay(OverlayBase):
    """
    Reads telemetry data from JSON file and displays live values.
    Uses a background thread to avoid blocking the render loop.
    """

    def __init__(self, data_source="telemetry.json", enabled=True,
                 position=(10, 80), font_scale=0.7, line_spacing=30,
                 update_interval=0.1):
        super().__init__(enabled)
        self.data_source = data_source
        self.position = position
        self.font_scale = font_scale
        self.line_spacing = line_spacing
        self.update_interval = update_interval  # How often to read file (seconds)

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.thickness = 2
        self.color = (255, 255, 255)  # White
        self.bg_color = (0, 0, 0)  # Black background for readability

        # Thread-safe telemetry storage
        self._lock = threading.Lock()
        self._velocity = 0.0
        self._acceleration = 0.0
        self._altitude = 0.0

        # Background reader thread
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _reader_loop(self):
        """Background thread that reads telemetry file periodically."""
        while not self._stop_event.is_set():
            if os.path.exists(self.data_source):
                try:
                    with open(self.data_source, "r") as f:
                        data = json.load(f)

                    with self._lock:
                        self._velocity = data.get("velocity", 0.0)
                        self._acceleration = data.get("acceleration", 0.0)
                        self._altitude = data.get("altitude", 0.0)
                except (json.JSONDecodeError, IOError):
                    # File might be being written to, skip this update
                    pass

            self._stop_event.wait(self.update_interval)

    def stop(self):
        """Stop the background reader thread."""
        self._stop_event.set()
        self._reader_thread.join(timeout=1.0)

    @property
    def velocity(self):
        with self._lock:
            return self._velocity

    @property
    def acceleration(self):
        with self._lock:
            return self._acceleration

    @property
    def altitude(self):
        with self._lock:
            return self._altitude

    def _draw_text_with_background(self, frame, text, position):
        """Draw text with a semi-transparent background for readability."""
        (text_width, text_height), baseline = cv2.getTextSize(
            text, self.font, self.font_scale, self.thickness
        )

        x, y = position
        padding = 5

        cv2.rectangle(
            frame,
            (x - padding, y - text_height - padding),
            (x + text_width + padding, y + baseline + padding),
            self.bg_color,
            -1
        )

        cv2.putText(frame, text, position, self.font, self.font_scale,
                    self.color, self.thickness)

    def render(self, frame, context=None):
        """Render telemetry data onto frame (no I/O, just reads from memory)."""
        x, y = self.position

        lines = [
            f"Velocity: {self.velocity:.2f} km/h",
            f"Accel: {self.acceleration:.2f} m/s^2",
            f"Altitude: {self.altitude:.2f} km",
        ]

        for i, line in enumerate(lines):
            pos = (x, y + i * self.line_spacing)
            self._draw_text_with_background(frame, line, pos)

        return frame
