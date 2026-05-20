#!/usr/bin/env python3
"""Zoom into takeoff transition to diagnose instability onset."""

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

    POS = 44; QUAT = 68
    LIN_VEL = 44 + 56 + 288  # 388
    ANG_VEL = LIN_VEL + 24   # 412

    n = len(rows)
    ts = np.zeros(n); pos = np.zeros((n, 3)); quat = np.zeros((n, 4))
    lv = np.zeros((n, 3)); av = np.zeros((n, 3))

    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        pos[i] = struct.unpack_from('<3d', data, POS)
        quat[i] = struct.unpack_from('<4d', data, QUAT)
        lv[i] = struct.unpack_from('<3d', data, LIN_VEL)
        av[i] = struct.unpack_from('<3d', data, ANG_VEL)

    norms = np.linalg.norm(quat, axis=1)
    v = norms > 0.9
    ts, pos, quat, lv, av = ts[v], pos[v], quat[v], lv[v], av[v]
    eul = Rotation.from_quat(quat).as_euler('xyz', degrees=True)
    return ts, pos, eul, lv, av

def parse_wrench(dbpath, topic):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=? ORDER BY m.timestamp', (topic,))
    rows = cur.fetchall()
    db.close()
    if not rows:
        return np.array([]), np.zeros((0, 3)), np.zeros((0, 3))

    data0 = rows[0][0]
    slen = struct.unpack_from('<I', data0, 12)[0]
    off = 16 + slen
    cdr_off = off - 4
    cdr_off = (cdr_off + 7) & ~7
    foff = cdr_off + 4

    n = len(rows)
    ts = np.zeros(n); f = np.zeros((n, 3)); t = np.zeros((n, 3))
    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        f[i] = struct.unpack_from('<3d', data, foff)
        t[i] = struct.unpack_from('<3d', data, foff + 24)
    return ts, f, t

def parse_rpm(dbpath):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=\"/uav/actual_rpm\" ORDER BY m.timestamp')
    rows = cur.fetchall()
    db.close()
    if not rows:
        return np.array([]), np.zeros((0, 6))
    n = len(rows)
    ts = np.zeros(n); rpms = np.zeros((n, 6))
    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        rpms[i] = struct.unpack_from('<6I', data, 20)
    return ts, rpms

def parse_cmd_raw(dbpath):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=\"/uav/cmd_raw\" ORDER BY m.timestamp')
    rows = cur.fetchall()
    db.close()
    if not rows:
        return np.array([]), np.zeros((0, 6))
    n = len(rows)
    ts = np.zeros(n); cmds = np.zeros((n, 6))
    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        cmds[i] = struct.unpack_from('<6H', data, 18)
    return ts, cmds


SIM_DB = '../2026_05_20_sim_issue/01_issue/01_issue_0.db3'
REAL_DB = '../2026_05_15_free_flight/eps_tau_0p1/01_data/01_data_0.db3'

# Parse data
s_t, s_pos, s_eul, s_lv, s_av = parse_odom(SIM_DB)
r_t, r_pos, r_eul, r_lv, r_av = parse_odom(REAL_DB)
s_ht, s_hf, s_htq = parse_wrench(SIM_DB, '/hgdo/wrench')
r_ht, r_hf, r_htq = parse_wrench(REAL_DB, '/hgdo/wrench')
s_ct, s_cf, s_ctq = parse_wrench(SIM_DB, '/nmpc/control')
r_ct, r_cf, r_ctq = parse_wrench(REAL_DB, '/nmpc/control')
s_rt, s_rpm = parse_rpm(SIM_DB)
r_rt, r_rpm = parse_rpm(REAL_DB)
s_cmt, s_cmd = parse_cmd_raw(SIM_DB)
r_cmt, r_cmd = parse_cmd_raw(REAL_DB)

# Align to start of arming (NMPC Fz first goes above idle 6.0)
def find_arm(ct, cf, thresh=6.1):
    idx = np.where(cf[:, 2] > thresh)[0]
    return ct[idx[0]] if len(idx) > 0 else ct[0]

s_arm = find_arm(s_ct, s_cf)
r_arm = find_arm(r_ct, r_cf)

for arr in [s_t, s_ht, s_ct, s_rt, s_cmt]:
    arr -= s_arm
for arr in [r_t, r_ht, r_ct, r_rt, r_cmt]:
    arr -= r_arm

print(f"Sim arm at t=0 (absolute {s_arm:.2f})")
print(f"Real arm at t=0 (absolute {r_arm:.2f})")
print(f"Sim duration: {s_t[-1]:.2f}s, Real duration: {r_t[-1]:.2f}s")

# Time window: focus on first 5s after arming
T0, T1 = -1, 5

fig, axes = plt.subplots(5, 2, figsize=(18, 20))
fig.suptitle('Takeoff Transition: Real vs Sim (aligned to arming)', fontsize=14, fontweight='bold')

labels_col = ['SIM (2026-05-20)', 'REAL (2026-05-15)']

for col, (ot, opos, oeul, olv, oav,
          cht, chf, chtq, cct, ccf, cctq,
          rt_arr, rpm_arr, cmt_arr, cmd_arr, col_label, color) in enumerate([
    (s_t, s_pos, s_eul, s_lv, s_av,
     s_ht, s_hf, s_htq, s_ct, s_cf, s_ctq,
     s_rt, s_rpm, s_cmt, s_cmd, 'SIM', 'b'),
    (r_t, r_pos, r_eul, r_lv, r_av,
     r_ht, r_hf, r_htq, r_ct, r_cf, r_ctq,
     r_rt, r_rpm, r_cmt, r_cmd, 'REAL', 'r')
]):
    # Row 0: Z + Roll/Pitch
    ax = axes[0, col]
    ax.plot(ot, opos[:, 2], '-', color=color, label='Z [m]', linewidth=1.5)
    ax2 = ax.twinx()
    ax2.plot(ot, oeul[:, 0], '--', color='green', label='Roll [deg]', linewidth=0.8, alpha=0.7)
    ax2.plot(ot, oeul[:, 1], '--', color='orange', label='Pitch [deg]', linewidth=0.8, alpha=0.7)
    ax.set_title(f'{col_label}: Altitude + Attitude')
    ax.set_ylabel('Z [m]')
    ax2.set_ylabel('Angle [deg]')
    ax.legend(loc='upper left', fontsize=7)
    ax2.legend(loc='upper right', fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

    # Row 1: NMPC control (Fz, torques)
    ax = axes[1, col]
    ax.plot(cct, ccf[:, 2], '-', color=color, label='NMPC Fz [N]', linewidth=1.2)
    ax2 = ax.twinx()
    ax2.plot(cct, cctq[:, 0], '--', color='green', label='NMPC τx', linewidth=0.8)
    ax2.plot(cct, cctq[:, 1], '--', color='orange', label='NMPC τy', linewidth=0.8)
    ax2.plot(cct, cctq[:, 2], '--', color='purple', label='NMPC τz', linewidth=0.8)
    ax.set_title(f'{col_label}: NMPC Control')
    ax.set_ylabel('Fz [N]')
    ax2.set_ylabel('Torque [Nm]')
    ax.legend(loc='upper left', fontsize=7)
    ax2.legend(loc='upper right', fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

    # Row 2: HGDO estimates
    ax = axes[2, col]
    ax.plot(cht, chf[:, 2], '-', color=color, label='HGDO Fz [N]', linewidth=1.2)
    ax2 = ax.twinx()
    ax2.plot(cht, chtq[:, 0], '--', color='green', label='HGDO τx', linewidth=0.8)
    ax2.plot(cht, chtq[:, 1], '--', color='orange', label='HGDO τy', linewidth=0.8)
    ax2.plot(cht, chtq[:, 2], '--', color='purple', label='HGDO τz', linewidth=0.8)
    ax.set_title(f'{col_label}: HGDO Estimated Disturbance')
    ax.set_ylabel('Fz [N]')
    ax2.set_ylabel('Torque [Nm]')
    ax.legend(loc='upper left', fontsize=7)
    ax2.legend(loc='upper right', fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

    # Row 3: Individual motor RPMs
    ax = axes[3, col]
    for m in range(6):
        ax.plot(rt_arr, rpm_arr[:, m], linewidth=0.7, alpha=0.8, label=f'M{m+1}')
    ax.set_title(f'{col_label}: Motor RPMs')
    ax.set_ylabel('RPM')
    ax.legend(ncol=3, fontsize=6)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

    # Row 4: Angular velocities
    ax = axes[4, col]
    ax.plot(ot, oav[:, 0], '-', color='green', label='ωx', linewidth=1)
    ax.plot(ot, oav[:, 1], '-', color='orange', label='ωy', linewidth=1)
    ax.plot(ot, oav[:, 2], '-', color='purple', label='ωz', linewidth=1)
    ax.set_title(f'{col_label}: Angular Velocity')
    ax.set_ylabel('ω [rad/s]')
    ax.set_xlabel('Time from arming [s]')
    ax.legend(fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('takeoff_zoom_comparison.png', dpi=150, bbox_inches='tight')
print("Saved takeoff_zoom_comparison.png")
