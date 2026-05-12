"""
Acceleration comparison: Mocap double-differentiation vs Odom velocity differentiation.

Mocap (PoseStamped) is in world frame.
Odom velocity is in world frame (ENU).

Both are differentiated with Savitzky-Golay smoothing, then converted to
body-frame specific force for comparison with IMU accelerometer.

  f_body = R^T · (a_inertial + g·e3)   (ENU: g·e3 = [0,0,+g])
"""

import sqlite3
import struct
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

CDR_BASE = 4
G = 9.81


def align_cdr(rel, a):
    return (rel + a - 1) & ~(a - 1)


def read_bag_topic(db_path, topic_name):
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute("SELECT id FROM topics WHERE name=?", (topic_name,))
    row = c.fetchone()
    if row is None:
        conn.close()
        return []
    topic_id = row[0]
    c.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    )
    rows = c.fetchall()
    conn.close()
    return rows


def parse_pose_stamped(data):
    rel = 0
    sec = struct.unpack_from('<I', data, CDR_BASE + rel)[0]; rel += 4
    nsec = struct.unpack_from('<I', data, CDR_BASE + rel)[0]; rel += 4
    slen = struct.unpack_from('<I', data, CDR_BASE + rel)[0]; rel += 4 + slen
    rel = align_cdr(rel, 8)
    px, py, pz = struct.unpack_from('<3d', data, CDR_BASE + rel); rel += 24
    qx, qy, qz, qw = struct.unpack_from('<4d', data, CDR_BASE + rel)
    return sec + nsec * 1e-9, px, py, pz, qx, qy, qz, qw


def parse_odom(data):
    rel = 0
    sec = struct.unpack_from('<I', data, CDR_BASE + rel)[0]; rel += 4
    nsec = struct.unpack_from('<I', data, CDR_BASE + rel)[0]; rel += 4
    slen = struct.unpack_from('<I', data, CDR_BASE + rel)[0]; rel += 4 + slen
    rel = align_cdr(rel, 4)
    slen2 = struct.unpack_from('<I', data, CDR_BASE + rel)[0]; rel += 4 + slen2
    rel = align_cdr(rel, 8)
    px, py, pz = struct.unpack_from('<3d', data, CDR_BASE + rel); rel += 24
    qx, qy, qz, qw = struct.unpack_from('<4d', data, CDR_BASE + rel); rel += 32
    rel += 288
    vx, vy, vz = struct.unpack_from('<3d', data, CDR_BASE + rel); rel += 24
    wx, wy, wz = struct.unpack_from('<3d', data, CDR_BASE + rel)
    return sec + nsec * 1e-9, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz


def parse_imu(data):
    base = 4
    sec = struct.unpack_from('<I', data, base)[0]
    nsec = struct.unpack_from('<I', data, base + 4)[0]
    slen = struct.unpack_from('<I', data, base + 8)[0]
    off = base + ((12 + slen + 7) & ~7)
    qx, qy, qz, qw = struct.unpack_from('<4d', data, off); off += 32 + 72
    wx, wy, wz = struct.unpack_from('<3d', data, off); off += 24 + 72
    ax, ay, az = struct.unpack_from('<3d', data, off)
    return sec + nsec * 1e-9, qx, qy, qz, qw, ax, ay, az


def smooth_diff(t, x, window=51, poly=3):
    """Differentiate with Savitzky-Golay smoothing."""
    dt = np.median(np.diff(t))
    x_smooth = savgol_filter(x, window, poly, axis=0)
    dx = savgol_filter(x, window, poly, deriv=1, delta=dt, axis=0)
    return dx, x_smooth


def main():
    bag_dir = Path("data_set/2026_05_05_free_flight")
    bag_path = str(bag_dir / "02_ct_1p255" / "02_ct_1p255_0.db3")
    save_dir = Path("data_set/trained_models")
    save_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Acceleration: Mocap vs Odom vs IMU")
    print("=" * 60)

    # --- Parse mocap ---
    print("\n[1] Parsing mocap (/S550/pose)...")
    mc_ts, mc_pos, mc_q = [], [], []
    for _, data in read_bag_topic(bag_path, "/S550/pose"):
        try:
            t, px, py, pz, qx, qy, qz, qw = parse_pose_stamped(data)
            mc_ts.append(t)
            mc_pos.append([px, py, pz])
            mc_q.append([qx, qy, qz, qw])
        except:
            continue
    mc_ts = np.array(mc_ts)
    mc_pos = np.array(mc_pos)
    mc_q = np.array(mc_q)
    print(f"  Mocap: {len(mc_ts)} samples, dt_med={np.median(np.diff(mc_ts))*1000:.1f} ms")

    # --- Parse odom ---
    print("[2] Parsing odom (/mavros/local_position/odom)...")
    od_ts, od_pos, od_vel, od_q = [], [], [], []
    for _, data in read_bag_topic(bag_path, "/mavros/local_position/odom"):
        try:
            t, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz = parse_odom(data)
            od_ts.append(t)
            od_pos.append([px, py, pz])
            od_vel.append([vx, vy, vz])
            od_q.append([qx, qy, qz, qw])
        except:
            continue
    od_ts = np.array(od_ts)
    od_pos = np.array(od_pos)
    od_vel = np.array(od_vel)
    od_q = np.array(od_q)
    print(f"  Odom: {len(od_ts)} samples, dt_med={np.median(np.diff(od_ts))*1000:.1f} ms")

    # --- Parse IMU ---
    print("[3] Parsing IMU (/mavros/imu/data)...")
    imu_ts, imu_q, imu_acc = [], [], []
    for _, data in read_bag_topic(bag_path, "/mavros/imu/data"):
        try:
            t, qx, qy, qz, qw, ax, ay, az = parse_imu(data)
            imu_ts.append(t)
            imu_q.append([qx, qy, qz, qw])
            imu_acc.append([ax, ay, az])
        except:
            continue
    imu_ts = np.array(imu_ts)
    imu_q = np.array(imu_q)
    imu_acc = np.array(imu_acc)
    print(f"  IMU: {len(imu_ts)} samples, dt_med={np.median(np.diff(imu_ts))*1000:.1f} ms")

    # --- Time alignment ---
    t0 = min(mc_ts[0], od_ts[0], imu_ts[0])
    mc_ts -= t0
    od_ts -= t0
    imu_ts -= t0

    t_start = 5.0
    t_end = min(mc_ts[-1], od_ts[-1], imu_ts[-1]) - 5.0
    print(f"\n[4] Analysis window: {t_start:.1f} - {t_end:.1f} s")

    # --- Mocap: double differentiate position (world frame) ---
    print("[5] Mocap: position → velocity → acceleration (world frame)...")
    mc_vel, mc_pos_s = smooth_diff(mc_ts, mc_pos, window=51, poly=3)
    mc_acc, mc_vel_s = smooth_diff(mc_ts, mc_vel, window=51, poly=3)
    print(f"  Smoothing window: 51 samples ({51 * np.median(np.diff(mc_ts))*1000:.0f} ms)")

    # --- Odom: differentiate velocity (world frame) ---
    print("[6] Odom: velocity → acceleration (world frame)...")
    od_acc, od_vel_s = smooth_diff(od_ts, od_vel, window=31, poly=3)

    # --- Common time grid (IMU rate) ---
    mask_imu = (imu_ts >= t_start) & (imu_ts <= t_end)
    t_common = imu_ts[mask_imu]
    imu_acc_c = imu_acc[mask_imu]
    imu_q_c = imu_q[mask_imu]
    N = len(t_common)

    # Interpolate mocap acceleration to common grid
    mc_acc_interp = np.zeros((N, 3))
    for ax in range(3):
        f = interp1d(mc_ts, mc_acc[:, ax], kind='linear', fill_value='extrapolate')
        mc_acc_interp[:, ax] = f(t_common)

    # Interpolate odom acceleration to common grid
    od_acc_interp = np.zeros((N, 3))
    for ax in range(3):
        f = interp1d(od_ts, od_acc[:, ax], kind='linear', fill_value='extrapolate')
        od_acc_interp[:, ax] = f(t_common)

    # Interpolate mocap quaternion
    mc_q_interp = np.zeros((N, 4))
    for ax in range(4):
        f = interp1d(mc_ts, mc_q[:, ax], kind='linear', fill_value='extrapolate')
        mc_q_interp[:, ax] = f(t_common)
    for i in range(N):
        mc_q_interp[i] /= np.linalg.norm(mc_q_interp[i])

    # --- Convert to body-frame specific force ---
    # f_body = R^T · (a_inertial + [0,0,g])   (ENU: gravity = [0,0,-g], specific force adds g)
    print("[7] Converting to body-frame specific force...")

    g_vec = np.array([0.0, 0.0, G])

    mc_sf_body = np.zeros((N, 3))
    od_sf_body = np.zeros((N, 3))
    for i in range(N):
        R = Rotation.from_quat(imu_q_c[i]).as_matrix()
        mc_sf_body[i] = R.T @ (mc_acc_interp[i] + g_vec)
        od_sf_body[i] = R.T @ (od_acc_interp[i] + g_vec)

    # --- Statistics ---
    print(f"\n{'=' * 60}")
    print(f"Body-frame specific force comparison (m/s²)")
    print(f"{'':8s} {'IMU std':>10s} {'Mocap std':>10s} {'Odom std':>10s} {'IMU-Mc std':>12s} {'IMU-Od std':>12s}")
    labels = ['ax', 'ay', 'az']
    for j in range(3):
        res_mc = imu_acc_c[:, j] - mc_sf_body[:, j]
        res_od = imu_acc_c[:, j] - od_sf_body[:, j]
        print(f"{labels[j]:8s} {np.std(imu_acc_c[:, j]):10.4f} {np.std(mc_sf_body[:, j]):10.4f} "
              f"{np.std(od_sf_body[:, j]):10.4f} {np.std(res_mc):12.4f} {np.std(res_od):12.4f}")

    # --- World frame comparison ---
    print(f"\nWorld-frame acceleration comparison (m/s²)")
    print(f"{'':8s} {'Mocap std':>10s} {'Odom std':>10s} {'Mc-Od std':>12s}")
    wlabels = ['ax_w', 'ay_w', 'az_w']
    for j in range(3):
        res = mc_acc_interp[:, j] - od_acc_interp[:, j]
        print(f"{wlabels[j]:8s} {np.std(mc_acc_interp[:, j]):10.4f} "
              f"{np.std(od_acc_interp[:, j]):10.4f} {np.std(res):12.4f}")

    # --- Plot ---
    print(f"\n[8] Generating plots...")
    t_plot = t_common - t_common[0]

    fig, axes = plt.subplots(4, 3, figsize=(18, 16))
    fig.suptitle('Acceleration Comparison: Mocap (2× diff) vs Odom (1× diff) vs IMU\n'
                 'Body-frame specific force: f = R^T·(a_world + g·e3)',
                 fontsize=13)

    axis_names_w = ['X (world)', 'Y (world)', 'Z (world)']
    axis_names_b = ['ax (body)', 'ay (body)', 'az (body)']

    # Row 0: World-frame acceleration
    for j in range(3):
        ax = axes[0, j]
        ax.plot(t_plot, mc_acc_interp[:, j], 'r-', alpha=0.6, lw=0.5, label='Mocap 2×diff')
        ax.plot(t_plot, od_acc_interp[:, j], 'b-', alpha=0.6, lw=0.5, label='Odom 1×diff')
        ax.set_title(f'World accel {axis_names_w[j]}')
        ax.set_ylabel('m/s²')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 1: World-frame residual (mocap - odom)
    for j in range(3):
        ax = axes[1, j]
        res = mc_acc_interp[:, j] - od_acc_interp[:, j]
        ax.plot(t_plot, res, 'k-', alpha=0.5, lw=0.5,
                label=f'Mocap−Odom (std={np.std(res):.3f})')
        ax.axhline(0, color='r', lw=0.5, ls='--')
        ax.set_title(f'World accel residual {axis_names_w[j]}')
        ax.set_ylabel('m/s²')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 2: Body-frame specific force
    for j in range(3):
        ax = axes[2, j]
        ax.plot(t_plot, imu_acc_c[:, j], 'g-', alpha=0.3, lw=0.3, label='IMU')
        ax.plot(t_plot, mc_sf_body[:, j], 'r-', alpha=0.6, lw=0.5, label='Mocap')
        ax.plot(t_plot, od_sf_body[:, j], 'b-', alpha=0.6, lw=0.5, label='Odom')
        ax.set_title(f'Body specific force {axis_names_b[j]}')
        ax.set_ylabel('m/s²')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 3: Body-frame residual (IMU - reference)
    for j in range(3):
        ax = axes[3, j]
        res_mc = imu_acc_c[:, j] - mc_sf_body[:, j]
        res_od = imu_acc_c[:, j] - od_sf_body[:, j]
        ax.plot(t_plot, res_mc, 'r-', alpha=0.4, lw=0.4,
                label=f'IMU−Mocap (std={np.std(res_mc):.3f})')
        ax.plot(t_plot, res_od, 'b-', alpha=0.4, lw=0.4,
                label=f'IMU−Odom (std={np.std(res_od):.3f})')
        ax.axhline(0, color='k', lw=0.5, ls='--')
        ax.set_title(f'Body residual {axis_names_b[j]}')
        ax.set_ylabel('m/s²')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    for j in range(3):
        axes[3, j].set_xlabel('Time (s)')

    plt.tight_layout()
    out_path = save_dir / "accel_mocap_vs_odom.png"
    plt.savefig(str(out_path), dpi=150)
    print(f"  Saved: {out_path}")
    print("\nDone!")


if __name__ == "__main__":
    main()
