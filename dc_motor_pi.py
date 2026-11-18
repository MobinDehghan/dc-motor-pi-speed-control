"""
DC motor speed control: open-loop vs. PI (simple Python simulation)

Motor equations:
    L di/dt = V - R*i - Ke*w        (electrical)
    J dw/dt = Kt*i - B*w - T_load   (mechanical)

A load torque is applied at t = 0.5 s. We compare:
  - open-loop: fixed voltage
  - closed-loop: PI controller that adjusts the voltage using the speed error

Motor parameters are example values for a small 12 V motor.
"""

import numpy as np
import matplotlib.pyplot as plt

# Motor parameters (example values)
R, L = 2.0, 1e-3          # resistance [ohm], inductance [H]
Ke = Kt = 0.05            # back-EMF and torque constants
J, B = 2e-5, 1e-5         # inertia, friction
V_MAX = 12.0              # supply limit [V]

# Simulation settings
dt = 1e-4                 # time step [s]
t = np.arange(0, 1.0, dt)
speed_ref = 150.0         # desired speed [rad/s]
load = 0.02               # load torque after t = 0.5 s [N.m]

# PI gains
Kp, Ki = 0.05, 2.5


def simulate(use_pi):
    i = 0.0               # current
    w = 0.0               # speed
    integral = 0.0        # integral of the speed error
    speeds = []

    # fixed voltage that gives the reference speed with no load
    v_fixed = speed_ref * (R * B + Ke * Kt) / Kt

    for time in t:
        T_load = load if time >= 0.5 else 0.0

        if use_pi:
            error = speed_ref - w
            integral += error * dt
            V = Kp * error + Ki * integral
            V = np.clip(V, -V_MAX, V_MAX)     # voltage limit
        else:
            V = v_fixed

        # motor equations (Euler method)
        i += (V - R * i - Ke * w) / L * dt
        w += (Kt * i - B * w - T_load) / J * dt
        speeds.append(w)

    return np.array(speeds)


open_loop = simulate(use_pi=False)
closed_loop = simulate(use_pi=True)

# Speed drop after the load is applied
after = t >= 0.5
print(f"Open-loop: speed drop = {speed_ref - open_loop[after].min():.1f} rad/s")
print(f"PI:        speed drop = {speed_ref - closed_loop[after].min():.1f} rad/s")
print(f"Final speed error (open-loop): {speed_ref - open_loop[-1]:.1f} rad/s")
print(f"Final speed error (PI):        {speed_ref - closed_loop[-1]:.1f} rad/s")

plt.plot(t, open_loop, label="Open-loop")
plt.plot(t, closed_loop, label="PI controller")
plt.axhline(speed_ref, ls="--", color="gray", label="Reference")
plt.xlabel("Time (s)")
plt.ylabel("Speed (rad/s)")
plt.title("Load applied at t = 0.5 s")
plt.legend()
plt.show()
