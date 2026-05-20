#!/usr/bin/env python3
"""Analyze angular dynamics: compare working sim (05-13), broken sim (05-20), and real (05-15)."""

import sqlite3
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def parse_odom_generic(dbpath, topic='/mavros/local_position/odom'):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=? ORDER BY m.timestamp', (topic,))
    rows = cur.fetchall()
    db.close()

    if not rows:
        return None

    # Detect position offset from frame_id length
    data0 = rows[0][0]
    slen = struct.unpack_from('<I', data0, 12)[0]
    off = 16 + slen
    off = (off + 3) & ~3
    slen2 = struct.unpack_from('<I', data0, off)[0]
    off = off + 4 + slen2
    cdr_off = off - 4
    cdr_off = (cdr_off + 7) & ~7
    POS = cdr_off + 4
    QUAT = POS + 24
    LIN_VEL = POS + 56 + 288
    ANG_VEL = LIN_VEL + 24

    n = len(rows)
    ts = np.zeros(n)
    pos = np.zeros((n, 3))
    quat = np.zeros((n, 4))
    lv = np.zeros((n, 3))
    av = np.zeros((n, 3))

    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        pos[i] = struct.unpack_from('<3d', data, POS)
        quat[i] = struct.unpack_from('<4d', data, QUAT)
        lv[i] = struct.unpack_from('<3d', data, LIN_VEL)
        av[i] = struct.unpack_from('<3d', data, ANG_VEL)

    norms = np.linalg.norm(quat, axis=1)
    v = norms > 0.9
    return {
        't': ts[v], 'pos': pos[v], 'quat': quat[v],
        'lv': lv[v], 'av': av[v],
        'pos_off': POS
    }


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
        return None

    data0 = rows[0][0]
    slen = struct.unpack_from('<I', data0, 12)[0]
    off = 16 + slen
    cdr_off = off - 4
    cdr_off = (cdr_off + 7) & ~7
    foff = cdr_off + 4

    n = len(rows)
    ts = np.zeros(n)
    force = np.zeros((n, 3))
    torque = np.zeros((n, 3))
    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        force[i] = struct.unpack_from('<3d', data, foff)
        torque[i] = struct.unpack_from('<3d', data, foff + 24)
    return {'t': ts, 'force': force, 'torque': torque}


datasets = {
    'SIM 05-13\n(working)': {
        'odom_db': '/home/user/drone_simulator_pkgs/data_set/2026_05_13_sim_flight/01_sim/01_sim_0.db3',
        'color': 'green',
    },
    'SIM 05-20\n(broken)': {
        'odom_db': '/home/user/drone_simulator_pkgs/data_set/2026_05_20_sim_issue/01_issue/01_issue_0.db3',
        'color': 'blue',
    },
    'REAL 05-15': {
        'odom_db': '/home/user/drone_simulator_pkgs/data_set/2026_05_15_free_flight/eps_tau_0p1/01_data/01_data_0.db3',
        'color': 'red',
    },
}

results = {}
for name, cfg in datasets.items():
    print(f"Parsing {name}...")
    odom = parse_odom_generic(cfg['odom_db'])
    hgdo = parse_wrench(cfg['odom_db'], '/hgdo/wrench')
    nmpc = parse_wrench(cfg['odom_db'], '/nmpc/control')

    if odom is None:
        print(f"  No odom data, skipping")
        continue

    # Compute angular acceleration (numerical derivative)
    dt = np.diff(odom['t'])
    dt[dt < 1e-6] = 1e-6
    alpha = np.diff(odom['av'], axis=0) / dt[:, None]

    # Find takeoff (NMPC Fz > 6.1)
    if nmpc is not None:
        arm_idx = np.where(nmpc['force'][:, 2] > 6.1)[0]
        t_arm = nmpc['t'][arm_idx[0]] if len(arm_idx) > 0 else odom['t'][0]
    else:
        t_arm = odom['t'][0]

    results[name] = {
        'odom': odom,
        'hgdo': hgdo,
        'nmpc': nmpc,
        'alpha': alpha,
        'alpha_t': odom['t'][:-1] + dt / 2,
        't_arm': t_arm,
        'color': cfg['color'],
    }
    print(f"  {len(odom['t'])} odom msgs, arm at t={t_arm:.2f}s")

# Plot
fig, axes = plt.subplots(4, 3, figsize=(20, 16))
fig.suptitle('Angular Dynamics: Working Sim (05-13) vs Broken Sim (05-20) vs Real (05-15)',
             fontsize=14, fontweight='bold')

for col, (name, r) in enumerate(results.items()):
    t_arm = r['t_arm']
    color = r['color']
    T0, T1 = -1, 5

    # Row 0: Angular velocity
    ax = axes[0, col]
    t = r['odom']['t'] - t_arm
    ax.plot(t, r['odom']['av'][:, 0], color='green', label='ωx', linewidth=0.8)
    ax.plot(t, r['odom']['av'][:, 1], color='orange', label='ωy', linewidth=0.8)
    ax.plot(t, r['odom']['av'][:, 2], color='purple', label='ωz', linewidth=0.8)
    ax.set_title(f'{name}')
    ax.set_ylabel('ω [rad/s]')
    ax.legend(fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

    # Row 1: Angular acceleration
    ax = axes[1, col]
    t = r['alpha_t'] - t_arm
    ax.plot(t, r['alpha'][:, 0], color='green', label='αx', linewidth=0.5, alpha=0.8)
    ax.plot(t, r['alpha'][:, 1], color='orange', label='αy', linewidth=0.5, alpha=0.8)
    ax.plot(t, r['alpha'][:, 2], color='purple', label='αz', linewidth=0.5, alpha=0.8)
    ax.set_ylabel('α [rad/s²]')
    ax.legend(fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

    # Row 2: HGDO torque
    ax = axes[2, col]
    if r['hgdo'] is not None:
        t = r['hgdo']['t'] - t_arm
        ax.plot(t, r['hgdo']['torque'][:, 0], color='green', label='HGDO τx', linewidth=0.8)
        ax.plot(t, r['hgdo']['torque'][:, 1], color='orange', label='HGDO τy', linewidth=0.8)
        ax.plot(t, r['hgdo']['torque'][:, 2], color='purple', label='HGDO τz', linewidth=0.8)
    ax.set_ylabel('Torque [Nm]')
    ax.legend(fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

    # Row 3: NMPC torque
    ax = axes[3, col]
    if r['nmpc'] is not None:
        t = r['nmpc']['t'] - t_arm
        ax.plot(t, r['nmpc']['torque'][:, 0], color='green', label='NMPC τx', linewidth=0.8)
        ax.plot(t, r['nmpc']['torque'][:, 1], color='orange', label='NMPC τy', linewidth=0.8)
        ax.plot(t, r['nmpc']['torque'][:, 2], color='purple', label='NMPC τz', linewidth=0.8)
    ax.set_ylabel('Torque [Nm]')
    ax.set_xlabel('Time from arming [s]')
    ax.legend(fontsize=7)
    ax.set_xlim(T0, T1)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('angular_dynamics_comparison.png', dpi=150, bbox_inches='tight')
print("Saved angular_dynamics_comparison.png")

# Print statistics for hover period (1-3s after arming)
print("\n=== Angular Dynamics Statistics (1-3s after arming) ===")
for name, r in results.items():
    t_arm = r['t_arm']
    t = r['odom']['t'] - t_arm
    mask = (t >= 1) & (t <= 3)

    if np.sum(mask) < 5:
        print(f"\n{name}: insufficient data in hover window")
        continue

    av = r['odom']['av'][mask]
    print(f"\n{name}:")
    print(f"  ω_x: mean={np.mean(av[:,0]):.4f}, std={np.std(av[:,0]):.4f}, max_abs={np.max(np.abs(av[:,0])):.4f}")
    print(f"  ω_y: mean={np.mean(av[:,1]):.4f}, std={np.std(av[:,1]):.4f}, max_abs={np.max(np.abs(av[:,1])):.4f}")
    print(f"  ω_z: mean={np.mean(av[:,2]):.4f}, std={np.std(av[:,2]):.4f}, max_abs={np.max(np.abs(av[:,2])):.4f}")

    if r['hgdo'] is not None:
        ht = r['hgdo']['t'] - t_arm
        hmask = (ht >= 1) & (ht <= 3)
        if np.sum(hmask) > 5:
            htq = r['hgdo']['torque'][hmask]
            print(f"  HGDO τx: mean={np.mean(htq[:,0]):.4f}, std={np.std(htq[:,0]):.4f}")
            print(f"  HGDO τy: mean={np.mean(htq[:,1]):.4f}, std={np.std(htq[:,1]):.4f}")
            print(f"  HGDO τz: mean={np.mean(htq[:,2]):.4f}, std={np.std(htq[:,2]):.4f}")
