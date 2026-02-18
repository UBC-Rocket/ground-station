#!/usr/bin/env python3
"""
Simple UDP video stream receiver and display
Receives H264/H265 stream from cosmostreamer and displays it in a window
Saves video to captures folder when stream ends
"""

import cv2
import numpy as np
import subprocess
import sys
import threading
import queue
import os
import signal
from datetime import datetime

from overlays import OverlayManager, StaticImageOverlay, StatusOverlay, TelemetryOverlay

# Global shutdown flag for clean exit from any context
shutdown_flag = threading.Event()

# Configuration
UDP_PORT = 3000
FRAME_WIDTH = 1920  # Adjust to match your camera resolution
FRAME_HEIGHT = 1080
FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT * 3  # 3 bytes per pixel (BGR)
CAPTURES_DIR = "captures"
VIDEO_FPS = 30  # Adjust based on your stream's FPS

# Initialize overlay system
overlay_manager = OverlayManager()
overlay_manager.add(StaticImageOverlay("overlay.png"))
overlay_manager.add(TelemetryOverlay("telemetry.json"))
overlay_manager.add(StatusOverlay())

def read_frames(process, frame_queue, recording_queue):
    """Read raw video frames from GStreamer subprocess"""
    while not shutdown_flag.is_set():
        try:
            raw_frame = process.stdout.read(FRAME_SIZE)
            if len(raw_frame) != FRAME_SIZE:
                print("Stream ended or incomplete frame")
                shutdown_flag.set()
                break

            # Convert raw bytes to numpy array (copy to make it writable)
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((FRAME_HEIGHT, FRAME_WIDTH, 3)).copy()

            # Put frame in display queue (non-blocking, drop old frames if queue is full)
            try:
                frame_queue.put_nowait(frame)
            except queue.Full:
                pass  # Drop frame if display queue is full

            # Put frame in recording queue (always save for recording)
            recording_queue.put(frame.copy())

        except Exception as e:
            print(f"Error reading frame: {e}")
            break

def save_recording(frames, start_time, end_time, frame_count, end_reason):
    """Save recorded frames to video file and metadata to txt file"""
    if not frames:
        print("No frames to save")
        return

    # Create captures directory if it doesn't exist
    os.makedirs(CAPTURES_DIR, exist_ok=True)

    # Generate timestamp for filename
    timestamp = start_time.strftime("%Y%m%d_%H%M%S")
    video_filename = os.path.join(CAPTURES_DIR, f"capture_{timestamp}.mp4")
    metadata_filename = os.path.join(CAPTURES_DIR, f"capture_{timestamp}.txt")

    print(f"\nSaving {len(frames)} frames to {video_filename}...")

    # Create video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_filename, fourcc, VIDEO_FPS, (FRAME_WIDTH, FRAME_HEIGHT))

    for frame in frames:
        video_writer.write(frame)

    video_writer.release()
    print(f"Video saved: {video_filename}")

    # Calculate metadata
    duration = (end_time - start_time).total_seconds()
    actual_fps = len(frames) / duration if duration > 0 else 0
    file_size = os.path.getsize(video_filename) if os.path.exists(video_filename) else 0

    # Save metadata
    metadata = f"""Capture Metadata
================
Filename: {os.path.basename(video_filename)}
Start Time: {start_time.strftime("%Y-%m-%d %H:%M:%S")}
End Time: {end_time.strftime("%Y-%m-%d %H:%M:%S")}
Duration: {duration:.2f} seconds
Total Frames: {len(frames)}
Displayed Frames: {frame_count}
Resolution: {FRAME_WIDTH}x{FRAME_HEIGHT}
Target FPS: {VIDEO_FPS}
Actual FPS: {actual_fps:.2f}
File Size: {file_size / (1024*1024):.2f} MB
UDP Port: {UDP_PORT}
End Reason: {end_reason}
"""

    with open(metadata_filename, 'w') as f:
        f.write(metadata)

    print(f"Metadata saved: {metadata_filename}")


def main():
    global shutdown_flag

    print("Starting video receiver...")
    print(f"Listening for UDP stream on port {UDP_PORT}")
    print(f"Expected resolution: {FRAME_WIDTH}x{FRAME_HEIGHT}")

    # GStreamer pipeline - outputs raw BGR frames to stdout
    gst_command = [
        'gst-launch-1.0.exe',
        '-q',  # Quiet mode
        'udpsrc',
        f'port={UDP_PORT}',
        'buffer-size=13000000',
        '!', 'parsebin',
        '!', 'decodebin',
        '!', 'videoconvert',
        '!', f'video/x-raw,format=BGR,width={FRAME_WIDTH},height={FRAME_HEIGHT}',
        '!', 'fdsink',  # Output to stdout (file descriptor sink)
    ]

    print("Starting GStreamer process...")

    frame_count = 0
    recorded_frames = []
    start_time = None
    end_reason = "Unknown"

    # Set up signal handlers for graceful shutdown
    def signal_handler(signum, frame):
        nonlocal end_reason
        print(f"\nReceived signal {signum}, shutting down...")
        end_reason = f"Signal {signum} received"
        shutdown_flag.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Start GStreamer as subprocess
        process = subprocess.Popen(
            gst_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=FRAME_SIZE
        )

        # Create queues for frames
        frame_queue = queue.Queue(maxsize=5)
        recording_queue = queue.Queue()

        # Start thread to read frames
        reader_thread = threading.Thread(target=read_frames, args=(process, frame_queue, recording_queue), daemon=True)
        reader_thread.start()

        print("Video stream opened successfully!")
        print("Press 'q' to quit, or close the window")
        print("Recording will be saved when stream ends...")

        # Create window
        window_name = 'Drone Video Feed'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        start_time = datetime.now()

        while not shutdown_flag.is_set():
            # Collect frames from recording queue
            while not recording_queue.empty():
                try:
                    recorded_frames.append(recording_queue.get_nowait())
                except queue.Empty:
                    break

            try:
                # Get frame from display queue (short timeout to stay responsive)
                frame = frame_queue.get(timeout=0.1)

                frame_count += 1

                # Apply overlays to frame
                context = {"frame_count": frame_count, "recording": True}
                frame = overlay_manager.render(frame, context)

                # Display the frame
                cv2.imshow(window_name, frame)

            except queue.Empty:
                # No frame received in timeout period
                if not reader_thread.is_alive():
                    print("Frame reader thread stopped")
                    end_reason = "Stream ended or connection lost"
                    break

            # Check for 'q' key or window close (must call waitKey for window events)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quit signal received (pressed 'q')")
                end_reason = "User quit (pressed 'q')"
                shutdown_flag.set()
                break

            # Check if window was closed via X button
            # WND_PROP_VISIBLE returns < 1 when window is closed
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    print("Window closed by user")
                    end_reason = "Window closed (X button)"
                    shutdown_flag.set()
                    break
            except cv2.error:
                # Window doesn't exist anymore
                print("Window no longer exists")
                end_reason = "Window closed"
                shutdown_flag.set()
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        end_reason = "Keyboard interrupt (Ctrl+C)"
        shutdown_flag.set()

    except FileNotFoundError:
        print("ERROR: gst-launch-1.0.exe not found")
        print("Make sure GStreamer is installed and in your PATH")
        sys.exit(1)

    finally:
        end_time = datetime.now()
        shutdown_flag.set()  # Ensure flag is set for cleanup

        # Give reader thread time to finish
        if 'reader_thread' in locals() and reader_thread.is_alive():
            print("Waiting for reader thread to finish...")
            reader_thread.join(timeout=2.0)

        # Collect any remaining frames from recording queue
        if 'recording_queue' in locals():
            while not recording_queue.empty():
                try:
                    recorded_frames.append(recording_queue.get_nowait())
                except queue.Empty:
                    break

        # Cleanup process
        if 'process' in locals():
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                print("Force killing GStreamer process...")
                process.kill()
                process.wait()

        cv2.destroyAllWindows()
        print(f"Total frames displayed: {frame_count}")
        print(f"Total frames recorded: {len(recorded_frames)}")

        # Save the recording
        if recorded_frames and start_time:
            save_recording(recorded_frames, start_time, end_time, frame_count, end_reason)

        print("Video receiver stopped")

if __name__ == "__main__":
    main()