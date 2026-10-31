"""Resume only the simulation clock during a stationary, drive-disabled mask probe."""
import os
import subprocess
import time

from beamngpy import BeamNGpy

probe = '/home/ly/town_memory_validation_20261005/probe_green_house_prompts.py'
beamng = BeamNGpy('127.0.0.1', 25252, socket_timeout=6).open(launch=False)
try:
    beamng.control.resume()
    time.sleep(1.0)
    subprocess.run(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python', probe],
                   env=os.environ.copy(), check=True, timeout=75)
finally:
    try:
        beamng.control.pause()
    finally:
        beamng.disconnect()
