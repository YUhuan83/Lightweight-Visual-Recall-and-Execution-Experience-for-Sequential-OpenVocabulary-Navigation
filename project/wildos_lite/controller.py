"""Pure-pursuit waypoint controller -> BeamNG control dict."""
import math

import numpy as np

from . import graph


def pure_pursuit(position_xy, forward_xy, goal_xy, gain=1.2, max_throttle=0.45,
                 slow_angle_deg=50.0, stop_distance_m=2.0, brake_distance_m=1.2,
                 steer_sign=1.0):
    d = np.asarray(goal_xy, float) - np.asarray(position_xy, float)
    distance = float(np.linalg.norm(d))
    f = graph.unit_xy(forward_xy)
    left = graph.left_of(f)
    heading_error = math.atan2(float(d @ left), float(d @ f))
    steering = float(np.clip(gain * heading_error, -1.0, 1.0)) * steer_sign
    if abs(math.degrees(heading_error)) > slow_angle_deg:
        throttle = 0.12
    elif distance < stop_distance_m:
        throttle = 0.0
    else:
        throttle = max_throttle
    brake = 1.0 if distance < brake_distance_m else 0.0
    return {"steering": steering, "throttle": float(throttle), "brake": float(brake),
            "distance_m": distance, "heading_error_rad": float(heading_error)}
