#!/usr/bin/env python3
"""Compare real flight (2026-05-15) vs simulator (2026-05-20) data."""

import sqlite3
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def parse_odom(dbpath, topic='/mavros/local_position/odom'):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=? ORDER BY m.timestamp', (topic,))
    rows = cur.fetchall()
    db.close()

    POS = 44
    QUAT = 68
    LIN_VEL = POS + 56 + 36 * 8      # 388
    ANG_VEL = LIN_VEL + 24            # 412

    n = len(rows)
    ts = np.zeros(n)
    pos = np.zeros((n, 3))
    quat_xyzw = np.zeros((n, 4))
    lin_vel = np.zeros((n, 3))
    ang_vel = np.zeros((n, 3))

    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        pos[i] = struct.unpack_from('<3d', data, POS)
        quat_xyzw[i] = struct.unpack_from('<4d', data, QUAT)
        lin_vel[i] = struct.unpack_from('<3d', data, LIN_VEL)
        ang_vel[i] = struct.unpack_from('<3d', data, ANG_VEL)

    # Filter valid quaternions
    norms = np.linalg.norm(quat_xyzw, axis=1)
    valid = norms > 0.9
    ts = ts[valid]
    pos = pos[valid]
    quat_xyzw = quat_xyzw[valid]
    lin_vel = lin_vel[valid]
    ang_vel = ang_vel[valid]

    # Convert to euler (roll, pitch, yaw)
    rot = Rotation.from_quat(quat_xyzw)
    euler = rot.as_euler('xyz', degrees=True)

    ts -= ts[0]
    return ts, pos, euler, lin_vel, ang_vel


def parse_wrench(dbpath, topic):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=? ORDER BY m.timestamp', (topic,))
    rows = cur.fetchall()
    db.close()

    # frame_id='base_link'(10) or 'pd_nmpc_att'(12) → both land force at 28
    # Try to detect:
    if not rows:
        return np.array([]), np.zeros((0, 3)), np.zeros((0, 3))

    data0 = rows[0][0]
    slen = struct.unpack_from('<I', data0, 12)[0]
    # After header(12) + len(4) + string(slen), align to 8
    off = 16 + slen
    cdr_off = off - 4
    cdr_off = (cdr_off + 7) & ~7
    force_off = cdr_off + 4

    n = len(rows)
    ts = np.zeros(n)
    force = np.zeros((n, 3))
    torque = np.zeros((n, 3))

    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        force[i] = struct.unpack_from('<3d', data, force_off)
        torque[i] = struct.unpack_from('<3d', data, force_off + 24)

    ts -= ts[0]
    return ts, force, torque


def parse_rpm(dbpath, topic='/uav/actual_rpm'):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=? ORDER BY m.timestamp', (topic,))
    rows = cur.fetchall()
    db.close()

    if not rows:
        return np.array([]), np.zeros((0, 6))

    n = len(rows)
    ts = np.zeros(n)
    rpms = np.zeros((n, 6))

    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        vals = struct.unpack_from('<6I', data, 20)
        rpms[i] = vals

    ts -= ts[0]
    return ts, rpms


SIM_DB = '../2026_05_20_sim_issue/01_issue/01_issue_0.db3'
REAL_DB = '../2026_05_15_free_flight/eps_tau_0p1/01_data/01_data_0.db3'

# Parse all data
print("Parsing sim odom...")
s_t, s_pos, s_eul, s_lv, s_av = parse_odom(SIM_DB)
print("Parsing real odom...")
r_t, r_pos, r_eul, r_lv, r_av = parse_odom(REAL_DB)

print("Parsing sim HGDO wrench...")
s_ht, s_hf, s_htq = parse_wrench(SIM_DB, '/hgdo/wrench')
print("Parsing real HGDO wrench...")
r_ht, r_hf, r_htq = parse_wrench(REAL_DB, '/hgdo/wrench')

print("Parsing sim NMPC control...")
s_ct, s_cf, s_ctq = parse_wrench(SIM_DB, '/nmpc/control')
print("Parsing real NMPC control...")
r_ct, r_cf, r_ctq = parse_wrench(REAL_DB, '/nmpc/control')

print("Parsing sim RPM...")
s_rt, s_rpm = parse_rpm(SIM_DB)
print("Parsing real RPM...")
r_rt, r_rpm = parse_rpm(REAL_DB)

# Align time to takeoff (z > 0.4m)
def find_takeoff(t, pos_z, threshold=0.4):
    idx = np.where(pos_z > threshold)[0]
    return t[idx[0]] if len(idx) > 0 else 0.0

s_takeoff = find_takeoff(s_t, s_pos[:, 2])
r_takeoff = find_takeoff(r_t, r_pos[:, 2])
print(f"Sim takeoff at t={s_takeoff:.2f}s, Real takeoff at t={r_takeoff:.2f}s")

# Shift all times relative to takeoff
s_t -= s_takeoff
s_ht -= s_takeoff if len(s_ht) else 0
s_ct -= s_takeoff if len(s_ct) else 0
s_rt -= s_takeoff if len(s_rt) else 0

r_t -= r_takeoff
r_ht -= r_takeoff if len(r_ht) else 0
r_ct -= r_takeoff if len(r_ct) else 0
r_rt -= r_takeoff if len(r_rt) else 0

# Time window around takeoff
T_MIN, T_MAX = -2, 10

fig, axes = plt.subplots(6, 2, figsize=(18, 24))
fig.suptitle('Real Flight (2026-05-15) vs Simulator (2026-05-20)', fontsize=16, fontweight='bold')

# ---- Column 0: Position & Attitude ----
# Row 0: Z position
ax = axes[0, 0]
ax.plot(s_t, s_pos[:, 2], 'b-', label='Sim Z', linewidth=1.2)
ax.plot(r_t, r_pos[:, 2], 'r-', label='Real Z', linewidth=1.2, alpha=0.8)
ax.set_ylabel('Z [m]')
ax.set_title('Altitude')
ax.legend()
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 1: Roll / Pitch
ax = axes[1, 0]
ax.plot(s_t, s_eul[:, 0], 'b-', label='Sim Roll', linewidth=1)
ax.plot(r_t, r_eul[:, 0], 'r-', label='Real Roll', linewidth=1, alpha=0.7)
ax.plot(s_t, s_eul[:, 1], 'b--', label='Sim Pitch', linewidth=1)
ax.plot(r_t, r_eul[:, 1], 'r--', label='Real Pitch', linewidth=1, alpha=0.7)
ax.set_ylabel('Angle [deg]')
ax.set_title('Roll & Pitch')
ax.legend(ncol=2, fontsize=8)
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 2: Angular velocity
ax = axes[2, 0]
ax.plot(s_t, s_av[:, 0], 'b-', label='Sim ωx', linewidth=1)
ax.plot(r_t, r_av[:, 0], 'r-', label='Real ωx', linewidth=1, alpha=0.7)
ax.plot(s_t, s_av[:, 1], 'b--', label='Sim ωy', linewidth=1)
ax.plot(r_t, r_av[:, 1], 'r--', label='Real ωy', linewidth=1, alpha=0.7)
ax.set_ylabel('ω [rad/s]')
ax.set_title('Angular Velocity (x, y)')
ax.legend(ncol=2, fontsize=8)
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 3: Yaw
ax = axes[3, 0]
ax.plot(s_t, s_eul[:, 2], 'b-', label='Sim Yaw', linewidth=1.2)
ax.plot(r_t, r_eul[:, 2], 'r-', label='Real Yaw', linewidth=1.2, alpha=0.8)
ax.set_ylabel('Yaw [deg]')
ax.set_title('Yaw')
ax.legend()
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 4: XY position
ax = axes[4, 0]
ax.plot(s_t, s_pos[:, 0], 'b-', label='Sim X', linewidth=1)
ax.plot(r_t, r_pos[:, 0], 'r-', label='Real X', linewidth=1, alpha=0.7)
ax.plot(s_t, s_pos[:, 1], 'b--', label='Sim Y', linewidth=1)
ax.plot(r_t, r_pos[:, 1], 'r--', label='Real Y', linewidth=1, alpha=0.7)
ax.set_ylabel('Position [m]')
ax.set_title('XY Position')
ax.legend(ncol=2, fontsize=8)
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 5: Linear velocity Z
ax = axes[5, 0]
ax.plot(s_t, s_lv[:, 2], 'b-', label='Sim Vz', linewidth=1.2)
ax.plot(r_t, r_lv[:, 2], 'r-', label='Real Vz', linewidth=1.2, alpha=0.8)
ax.set_ylabel('Vz [m/s]')
ax.set_title('Vertical Velocity')
ax.legend()
ax.set_xlim(T_MIN, T_MAX)
ax.set_xlabel('Time from takeoff [s]')
ax.grid(True, alpha=0.3)

# ---- Column 1: Control & Disturbance ----
# Row 0: NMPC Fz
ax = axes[0, 1]
ax.plot(s_ct, s_cf[:, 2], 'b-', label='Sim NMPC Fz', linewidth=1.2)
ax.plot(r_ct, r_cf[:, 2], 'r-', label='Real NMPC Fz', linewidth=1.2, alpha=0.8)
ax.set_ylabel('Fz [N]')
ax.set_title('NMPC Control Fz')
ax.legend()
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 1: NMPC torque
ax = axes[1, 1]
ax.plot(s_ct, s_ctq[:, 0], 'b-', label='Sim τx', linewidth=1)
ax.plot(r_ct, r_ctq[:, 0], 'r-', label='Real τx', linewidth=1, alpha=0.7)
ax.plot(s_ct, s_ctq[:, 1], 'b--', label='Sim τy', linewidth=1)
ax.plot(r_ct, r_ctq[:, 1], 'r--', label='Real τy', linewidth=1, alpha=0.7)
ax.set_ylabel('Torque [Nm]')
ax.set_title('NMPC Control Torque')
ax.legend(ncol=2, fontsize=8)
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 2: HGDO Fz
ax = axes[2, 1]
ax.plot(s_ht, s_hf[:, 2], 'b-', label='Sim HGDO Fz', linewidth=1.2)
ax.plot(r_ht, r_hf[:, 2], 'r-', label='Real HGDO Fz', linewidth=1.2, alpha=0.8)
ax.set_ylabel('Fz [N]')
ax.set_title('HGDO Estimated Fz')
ax.legend()
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 3: HGDO torque
ax = axes[3, 1]
ax.plot(s_ht, s_htq[:, 0], 'b-', label='Sim HGDO τx', linewidth=1)
ax.plot(r_ht, r_htq[:, 0], 'r-', label='Real HGDO τx', linewidth=1, alpha=0.7)
ax.plot(s_ht, s_htq[:, 1], 'b--', label='Sim HGDO τy', linewidth=1)
ax.plot(r_ht, r_htq[:, 1], 'r--', label='Real HGDO τy', linewidth=1, alpha=0.7)
ax.set_ylabel('Torque [Nm]')
ax.set_title('HGDO Estimated Torque')
ax.legend(ncol=2, fontsize=8)
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 4: RPM (mean of 6 motors)
ax = axes[4, 1]
if len(s_rt) > 0:
    ax.plot(s_rt, np.mean(s_rpm, axis=1), 'b-', label='Sim Mean RPM', linewidth=1.2)
if len(r_rt) > 0:
    ax.plot(r_rt, np.mean(r_rpm, axis=1), 'r-', label='Real Mean RPM', linewidth=1.2, alpha=0.8)
ax.set_ylabel('RPM')
ax.set_title('Mean Motor RPM')
ax.legend()
ax.set_xlim(T_MIN, T_MAX)
ax.grid(True, alpha=0.3)

# Row 5: RPM spread (max - min)
ax = axes[5, 1]
if len(s_rt) > 0:
    s_rpm_spread = np.max(s_rpm, axis=1) - np.min(s_rpm, axis=1)
    ax.plot(s_rt, s_rpm_spread, 'b-', label='Sim RPM spread', linewidth=1.2)
if len(r_rt) > 0:
    r_rpm_spread = np.max(r_rpm, axis=1) - np.min(r_rpm, axis=1)
    ax.plot(r_rt, r_rpm_spread, 'r-', label='Real RPM spread', linewidth=1.2, alpha=0.8)
ax.set_ylabel('RPM')
ax.set_title('Motor RPM Spread (max-min)')
ax.legend()
ax.set_xlim(T_MIN, T_MAX)
ax.set_xlabel('Time from takeoff [s]')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('real_vs_sim_comparison.png', dpi=150, bbox_inches='tight')
print("Saved real_vs_sim_comparison.png")
