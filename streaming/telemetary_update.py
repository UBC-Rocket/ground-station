"""Mock telemetry data generator. Outputs JSON for overlay system."""

import json
import random
import time

FILE_PATH = "telemetry.json"  # JSON format for overlay system

while True:
    velocity = random.uniform(0, 2400)  # km/h
    acceleration = random.uniform(0, 300)  # m/s^2
    altitude = random.uniform(0, 200)  # km

    with open(FILE_PATH, "w") as f:
        json.dump({
            "velocity": velocity,
            "acceleration": acceleration,
            "altitude": altitude
        }, f)

    time.sleep(0.5)
