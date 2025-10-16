import math
import numpy as np

# --- Winch and motion parameters ---
distance_per_leg = 1800.0    # in inches (150 ft)
accel = 26.7                 # in/s^2
radius_speed = 0.72          # in (used for speed calc)
radius_torque = 0.281        # in (used for torque calc)

# --- Payload weights per leg (oz) ---
payload_weights = [26, 18, 18, 12.5]

# --- Motor torque-speed parameters ---
T_hold = 84.0        # oz·in (holding torque)
RPM_drop = 1200.0    # RPM where torque decays significantly (tune to motor/driver)

# --- Desired cruise speed ---
cruise_speed = 90.0  # in/s (adjust to meet time target)

# --- Helper functions ---
def speed_to_rpm(v_in_per_s):
    return (v_in_per_s * 60) / (2 * math.pi * radius_speed)

def torque_available(rpm):
    return T_hold * math.exp(-rpm / RPM_drop)

def torque_required(payload_oz):
    return payload_oz * radius_torque

# --- Motion per leg ---
rpm_cruise = speed_to_rpm(cruise_speed)
t_accel = cruise_speed / accel
d_accel = 0.5 * accel * t_accel**2
d_cruise = distance_per_leg - 2*d_accel
t_cruise = d_cruise / cruise_speed
t_leg = 2*t_accel + t_cruise

# --- Results per leg ---
print(f"Cruise speed: {cruise_speed} in/s")
print(f"Cruise RPM: {rpm_cruise:.1f} RPM")
print(f"Per-leg time: {t_leg:.2f} s, Total time (4 legs): {t_leg*4:.2f} s\n")

T_avail = torque_available(rpm_cruise)
print(f"Estimated available torque at {rpm_cruise:.0f} RPM: {T_avail:.1f} oz·in\n")

for i, w in enumerate(payload_weights, start=1):
    T_req = torque_required(w)
    margin = T_avail - T_req
    status = "✅ OK" if margin >= 0 else "❌ TOO HIGH"
    print(f"Leg {i}: weight={w} oz | Required torque={T_req:.2f} oz·in | "
          f"Available={T_avail:.2f} | Margin={margin:.2f} | {status}")
