# -*- coding: utf-8 -*-
"""
Guidance logic (trail/lead, speed/altitude match, corridor, collision avoidance).
Teknofest hız sınırlarına uygun; mesafe 15 m'nin üzerinde yakalama için sürekli
hızlanma önceliklidir, 15 m civarında çarpışmadan kaçınmak için hız yumuşatılır.
"""

import math
from collections import deque
from typing import Optional

import numpy as np

try:
    from parametreler import *  # noqa
except ImportError:
    # Defaults in case parametreler.py yoksa
    D_TRAIL_FAR = 50.0
    D_TRAIL_NEAR = 20.0
    D_SAFE = 22.0
    D_VISION = 35.0
    K_SPEED = 1.2
    V_MIN = 10.0
    V_MAX = 30.0
    V_MAX_CATCHUP = 35.0
    K_ALTITUDE = 1.0
    VZ_MAX = 6.0
    T_LEAD_MIN = 0.5
    T_LEAD_MAX = 2.0
    ALPHA_LEAD = 0.6
    D_CATCH = 15.0
    GEOFENCE_RADIUS = None
    GEOFENCE_CENTER = None
else:
    if "D_CATCH" not in globals():
        D_CATCH = 15.0
    if "GEOFENCE_RADIUS" not in globals():
        GEOFENCE_RADIUS = None
    if "GEOFENCE_CENTER" not in globals():
        GEOFENCE_CENTER = None


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


class GuidanceLaw:
    """Trail + hız/irtifa eşleme + koridor + çarpışma kaçınma."""

    def __init__(self):
        self.d_trail_far = D_TRAIL_FAR
        self.d_trail_near = D_TRAIL_NEAR
        self.d_safe = D_SAFE
        self.d_vision = D_VISION
        self.k_speed = K_SPEED
        self.v_min = V_MIN
        # Komut ve param tarafında hız tavanını 30 m/s ile sınırla
        cap = 30.0
        self.v_max = min(V_MAX, cap)
        self.v_max_catchup = min(V_MAX_CATCHUP if "V_MAX_CATCHUP" in globals() else V_MAX, cap)
        self.k_altitude = K_ALTITUDE
        self.vz_max = VZ_MAX
        self.t_lead_min = T_LEAD_MIN
        self.t_lead_max = T_LEAD_MAX
        self.t_lead = 1.0
        self.alpha_lead = ALPHA_LEAD
        self.d_catch = D_CATCH if "D_CATCH" in globals() else 15.0
        self.trail_history = deque(maxlen=2)
        self.geofence_active = False

    def check_geofence(self, own_pos_ned):
        if not GEOFENCE_RADIUS:
            return None
        dist_from_center = np.linalg.norm(own_pos_ned[:2])
        if dist_from_center > GEOFENCE_RADIUS:
            self.geofence_active = True
            return -own_pos_ned[:2] / dist_from_center
        if self.geofence_active and dist_from_center < (GEOFENCE_RADIUS - 50.0):
            self.geofence_active = False
        return None

    def update_lead_time(self, distance, relative_vel_mag):
        if relative_vel_mag < 1.0:
            t_lead_raw = self.t_lead_max
        else:
            t_lead_raw = distance / relative_vel_mag
        t_lead_raw = clamp(t_lead_raw, self.t_lead_min, self.t_lead_max)
        self.t_lead = self.alpha_lead * self.t_lead + (1 - self.alpha_lead) * t_lead_raw

    def _adaptive_trail_distance(self, distance, target_speed):
        base = clamp(0.8 * distance, self.d_trail_near, self.d_trail_far)
        if target_speed > 15.0:
            base = clamp(base + 10.0, self.d_trail_near, self.d_trail_far + 10.0)
        return base

    def compute_trail_point(self, target_pos_ned, target_vel, distance, own_pos_ned, corridor_angle_deg=35.0, target_acc=None):
        if target_acc is not None:
            t_pred = clamp(distance / 20.0, 0.5, 2.0)
            target_pos_ned = target_pos_ned + np.append(target_vel[:2] * t_pred, 0) + np.append(0.5 * target_acc[:2] * t_pred**2, 0)
            target_vel = target_vel + np.append(target_acc[:2] * t_pred, 0)

        v_mag = np.linalg.norm(target_vel[:2])
        v_hat = np.array([1.0, 0.0]) if v_mag < 0.5 else target_vel[:2] / v_mag

        d_trail = self._adaptive_trail_distance(distance, v_mag)
        trail_xy = target_pos_ned[:2] - v_hat * d_trail
        trail_z = target_pos_ned[2]

        fence_vector = self.check_geofence(own_pos_ned)
        if self.geofence_active and fence_vector is not None:
            trail_xy = own_pos_ned[:2] + fence_vector * 60.0
            trail_point = np.array([trail_xy[0], trail_xy[1], trail_z])
            return trail_point, 0.0, False

        to_follower = own_pos_ned[:2] - target_pos_ned[:2]
        behind = False
        lateral_offset = np.zeros(2)
        if np.linalg.norm(to_follower) > 0.1:
            to_follower_norm = to_follower / np.linalg.norm(to_follower)
            angle_error = math.degrees(math.acos(np.clip(np.dot(-v_hat, to_follower_norm), -1, 1)))
            behind = angle_error < corridor_angle_deg
            if not behind:
                side = np.array([-v_hat[1], v_hat[0]])
                lateral_offset = side * clamp(angle_error / 90.0, 0.0, 1.0) * 10.0
                if angle_error > 20.0:
                    d_trail = max(self.d_trail_near, d_trail * 0.6)
                    trail_xy = target_pos_ned[:2] - v_hat * d_trail
        trail_xy = trail_xy + lateral_offset

        trail_point = np.array([trail_xy[0], trail_xy[1], trail_z])
        self.trail_history.append(trail_point)
        smoothed = np.mean(self.trail_history, axis=0) if len(self.trail_history) > 1 else trail_point
        return smoothed, d_trail, behind

    def compute_speed_command(self, distance, target_speed, target_acc=None, closing_rate=None):
        catch_dist = self.d_catch

        accel_boost = 0.0
        if target_acc is not None:
            acc_mag = np.linalg.norm(target_acc[:2])
            accel_boost = 1.2 * acc_mag

        closing_boost = 0.0
        if closing_rate is not None and closing_rate < 0:
            closing_boost = abs(closing_rate) * 1.5

        if distance > catch_dist:
            base_catchup = target_speed + 8.0
            if accel_boost > 0:
                base_catchup += accel_boost
            if closing_boost > 0:
                base_catchup += closing_boost

            if distance > 80.0:
                speed_cmd = self.v_max_catchup
            elif distance > 50.0:
                speed_cmd = max(base_catchup, self.v_max_catchup * 0.95)
            elif distance > 30.0:
                speed_cmd = max(base_catchup, self.v_max_catchup * 0.90)
            else:
                speed_cmd = max(base_catchup, self.v_max * 0.85)

            speed_cmd = max(speed_cmd, target_speed + 5.0)
            speed_cmd = max(speed_cmd, self.v_min + 12.0)
            v_max_eff = self.v_max_catchup
        else:
            match_speed = max(target_speed + 0.5, self.v_min)

            if distance < self.d_safe:
                safety_factor = clamp((distance - 10.0) / max(self.d_safe - 10.0, 1.0), 0.0, 1.0)
                speed_cmd = match_speed * safety_factor + self.v_min * (1 - safety_factor)
            elif distance < (catch_dist - 2.0):
                blend = clamp((distance - (catch_dist - 5.0)) / 3.0, 0.0, 1.0)
                speed_cmd = match_speed + blend * 2.0
            else:
                speed_cmd = match_speed + 1.5

            v_max_eff = self.v_max

        return float(np.clip(speed_cmd, self.v_min, v_max_eff))

    def compute_altitude_command(self, target_alt, current_alt):
        alt_error = target_alt - current_alt
        step = np.clip(self.k_altitude * alt_error, -self.vz_max, self.vz_max)
        return float(current_alt + step)

    def repulsive(self, own_pos_ned, target_pos_ned):
        to_target = own_pos_ned[:2] - target_pos_ned[:2]
        dist = np.linalg.norm(to_target)
        if dist < 1e-3 or dist >= self.d_safe:
            return np.zeros(2)
        strength = (1.0 / max(dist, 1.0) - 1.0 / self.d_safe) * 15.0
        return (to_target / dist) * strength
