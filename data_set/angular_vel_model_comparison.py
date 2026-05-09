"""
Angular Velocity & Attitude Model Comparison
=============================================
Computes motor torque from actual RPM, integrates angular dynamics (one-step),
and compares predicted angular velocity/attitude with IMU and EKF2 odom.

Also estimates CoM offset from mean HGDO disturbance torque during hover.

Method:
  1. CoM offset: dx = mean(τy_hgdo) / mean(Fz),  dy = -mean(τx_hgdo) / mean(Fz)
  2. Motor torque: τ = Σ[ (r_i - r_com) × [0,0,Fi] + [0,0, ±Cm·Fi] ]
  3. One-step: ω_pred(t+dt) = ω_imu(t) + I⁻¹[τ - ω×Iω - b·ω] · dt
"""

import sqlite3
import struct
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from scipy.signal import welch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

CDR_BASE = 4


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


def parse_imu(data):
    base = 4
    sec = struct.unpack_from('<I', data, base)[0]
    nsec = struct.unpack_from('<I', data, base + 4)[0]
    slen = struct.unpack_from('<I', data, base + 8)[0]
    off = base + ((12 + slen + 7) & ~7)
    qx, qy, qz, qw = struct.unpack_from('<4d', data, off)
    off += 32 + 72
    wx, wy, wz = struct.unpack_from('<3d', data, off)
    return sec + nsec * 1e-9, wx, wy, wz, qx, qy, qz, qw


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


def parse_wrench(data):
    base = 4
    sec = struct.unpack_from('<I', data, base)[0]
    nsec = struct.unpack_from('<I', data, base + 4)[0]
    slen = struct.unpack_from('<I', data, base + 8)[0]
    off = base + ((12 + slen + 7) & ~7)
    fx, fy, fz = struct.unpack_from('<3d', data, off); off += 24
    tx, ty, tz = struct.unpack_from('<3d', data, off)
    return sec + nsec * 1e-9, fx, fy, fz, tx, ty, tz


def parse_rpm(data):
    off = 4
    sec = struct.unpack_from('<I', data, off)[0]; off += 4
    nsec = struct.unpack_from('<I', data, off)[0]; off += 4
    fl = struct.unpack_from('<I', data, off)[0]; off += 4 + fl
    while off % 4:
        off += 1
    vals = struct.unpack_from('<6i', data, off)
    return sec + nsec * 1e-9, vals


# ============================================================
# Physical parameters
# ============================================================
CT = 1.255e-7
MOMENT_CONST = 0.01569
MASS = 3.188
G = 9.81

IXX = 0.06573874618
IYY = 0.06535731789
IZZ = 0.10317211827
I_DIAG = np.array([IXX, IYY, IZZ])

BX, BY, BZ = 0.115, 0.137, 0.184

MOTOR_POS = np.array([
    [0.2295, 0.1325, 0.062],    # M1: front left, CCW
    [0.0, 0.2650, 0.062],       # M2: left, CW
    [-0.2295, 0.1325, 0.062],   # M3: back left, CCW
    [-0.2295, -0.1325, 0.062],  # M4: back right, CW
    [0.0, -0.2650, 0.062],      # M5: right, CCW
    [0.2295, -0.1325, 0.062],   # M6: front right, CW
])

REACTION_SIGN = np.array([-1, +1, -1, +1, -1, +1])


def estimate_com_offset(bag_path):
    """Estimate CoM offset from mean HGDO torque during hover.

    τ_hgdo = r_com × [0, 0, Fz]
      τx = -dy·Fz  →  dy = -τx_mean / Fz_mean
      τy =  dx·Fz  →  dx =  τy_mean / Fz_mean
    """
    hgdo_rows = read_bag_topic(bag_path, "/hgdo/wrench")
    tx_list, ty_list = [], []
    for _, data in hgdo_rows:
        try:
            _, fx, fy, fz, tx, ty, tz = parse_wrench(data)
            tx_list.append(tx)
            ty_list.append(ty)
        except:
            continue
    tx_arr = np.array(tx_list)
    ty_arr = np.array(ty_list)

    rpm_rows = read_bag_topic(bag_path, "/uav/actual_rpm")
    fz_list = []
    for _, data in rpm_rows:
        try:
            _, vals = parse_rpm(data)
            fz_list.append(CT * sum(v ** 2 for v in vals))
        except:
            continue
    fz_arr = np.array(fz_list)

    n = len(tx_arr)
    s, e = n // 4, 3 * n // 4
    mean_tx = np.mean(tx_arr[s:e])
    mean_ty = np.mean(ty_arr[s:e])

    n2 = len(fz_arr)
    s2, e2 = n2 // 4, 3 * n2 // 4
    mean_fz = np.mean(fz_arr[s2:e2])

    dx = mean_ty / mean_fz
    dy = -mean_tx / mean_fz

    print(f"CoM offset estimation:")
    print(f"  mean τx_hgdo = {mean_tx:.6f} Nm")
    print(f"  mean τy_hgdo = {mean_ty:.6f} Nm")
    print(f"  mean Fz      = {mean_fz:.3f} N")
    print(f"  dx = {dx * 1000:.2f} mm,  dy = {dy * 1000:.2f} mm")

    return np.array([dx, dy, 0.0])


def main():
    bag_dir = Path("data_set/2026_05_05_free_flight")
    bag_path = str(bag_dir / "02_ct_1p255" / "02_ct_1p255_0.db3")
    save_dir = Path("data_set/trained_models")
    save_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Angular Velocity Model vs IMU vs EKF2 Odom")
    print("=" * 60)

    # --- Estimate CoM offset ---
    print("\n[1] Estimating CoM offset from HGDO torque...")
    com = estimate_com_offset(bag_path)
    r_motors = MOTOR_POS - com

    # --- Parse data ---
    print("\n[2] Parsing data...")
    imu_ts, imu_w, imu_q = [], [], []
    for _, data in read_bag_topic(bag_path, "/mavros/imu/data"):
        try:
            t, wx, wy, wz, qx, qy, qz, qw = parse_imu(data)
            imu_ts.append(t)
            imu_w.append([wx, wy, wz])
            imu_q.append([qx, qy, qz, qw])
        except:
            continue
    imu_ts = np.array(imu_ts)
    imu_w = np.array(imu_w)
    imu_q = np.array(imu_q)
    print(f"  IMU: {len(imu_ts)} samples")

    odom_ts, odom_w, odom_q = [], [], []
    for _, data in read_bag_topic(bag_path, "/mavros/local_position/odom"):
        try:
            t, *_, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz = parse_odom(data)
            odom_ts.append(t)
            odom_w.append([wx, wy, wz])
            odom_q.append([qx, qy, qz, qw])
        except:
            continue
    odom_ts = np.array(odom_ts)
    odom_w = np.array(odom_w)
    odom_q = np.array(odom_q)
    print(f"  Odom: {len(odom_ts)} samples")

    rpm_ts, rpm_vals = [], []
    for _, data in read_bag_topic(bag_path, "/uav/actual_rpm"):
        try:
            t, vals = parse_rpm(data)
            rpm_ts.append(t)
            rpm_vals.append(vals)
        except:
            continue
    rpm_ts = np.array(rpm_ts)
    rpm_vals = np.array(rpm_vals, dtype=np.float64)
    print(f"  RPM: {len(rpm_ts)} samples")

    # --- Time alignment ---
    t0 = min(imu_ts[0], rpm_ts[0], odom_ts[0])
    imu_ts -= t0
    rpm_ts -= t0
    odom_ts -= t0
    t_start = 5.0
    t_end = min(imu_ts[-1], rpm_ts[-1], odom_ts[-1]) - 5.0

    rpm_interp = np.zeros((len(imu_ts), 6))
    for m in range(6):
        f = interp1d(rpm_ts, rpm_vals[:, m], kind='previous', fill_value='extrapolate')
        rpm_interp[:, m] = f(imu_ts)

    mask = (imu_ts >= t_start) & (imu_ts <= t_end)
    t_sim = imu_ts[mask]
    rpms = rpm_interp[mask]
    w_imu = imu_w[mask]
    q_imu = imu_q[mask]
    N = len(t_sim)
    print(f"\n[3] Hover window: {t_start:.1f}-{t_end:.1f}s, {N} samples")

    # --- Compute motor torques ---
    print("[4] Computing motor torques...")
    torques = np.zeros((N, 3))
    for i in range(N):
        tau = np.zeros(3)
        for m in range(6):
            F_m = CT * rpms[i, m] ** 2
            tau += np.cross(r_motors[m], [0, 0, F_m])
            tau[2] += REACTION_SIGN[m] * MOMENT_CONST * F_m
        torques[i] = tau

    # --- One-step angular velocity prediction ---
    print("[5] One-step angular velocity prediction...")

    def ang_dyn(omega, tau):
        return (tau - np.cross(omega, I_DIAG * omega)
                - np.array([BX, BY, BZ]) * omega) / I_DIAG

    omega_pred = np.zeros((N, 3))
    omega_pred[0] = w_imu[0]
    for i in range(1, N):
        dt = t_sim[i] - t_sim[i - 1]
        if dt <= 0 or dt > 0.05:
            omega_pred[i] = w_imu[i]
            continue
        wdot = ang_dyn(w_imu[i - 1], torques[i - 1])
        omega_pred[i] = w_imu[i - 1] + wdot * dt

    # --- One-step attitude prediction ---
    print("[6] One-step attitude prediction...")

    def quat_step(q, omega, dt):
        wnorm = np.linalg.norm(omega)
        if wnorm < 1e-10:
            return q.copy()
        ha = 0.5 * wnorm * dt
        s = np.sin(ha) / wnorm
        dq = np.array([s * omega[0], s * omega[1], s * omega[2], np.cos(ha)])
        x1, y1, z1, w1 = q
        x2, y2, z2, w2 = dq
        qn = np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ])
        return qn / np.linalg.norm(qn)

    q_pred = np.zeros((N, 4))
    q_pred[0] = q_imu[0]
    for i in range(1, N):
        dt = t_sim[i] - t_sim[i - 1]
        if dt <= 0 or dt > 0.05:
            q_pred[i] = q_imu[i]
            continue
        q_pred[i] = quat_step(q_imu[i - 1], omega_pred[i - 1], dt)

    euler_pred = Rotation.from_quat(q_pred).as_euler('xyz', degrees=True)
    euler_imu = Rotation.from_quat(q_imu).as_euler('xyz', degrees=True)

    # --- Odom interpolation ---
    odom_w_interp = np.zeros((N, 3))
    for ax in range(3):
        f = interp1d(odom_ts, odom_w[:, ax], kind='linear', fill_value='extrapolate')
        odom_w_interp[:, ax] = f(t_sim)

    odom_q_interp = np.zeros((N, 4))
    for ax in range(4):
        f = interp1d(odom_ts, odom_q[:, ax], kind='linear', fill_value='extrapolate')
        odom_q_interp[:, ax] = f(t_sim)
    for i in range(N):
        odom_q_interp[i] /= np.linalg.norm(odom_q_interp[i])
    euler_odom = Rotation.from_quat(odom_q_interp).as_euler('xyz', degrees=True)

    # --- Results ---
    res_w_imu = w_imu - omega_pred
    res_w_odom = odom_w_interp - omega_pred
    res_att_imu = euler_imu - euler_pred
    res_att_odom = euler_odom - euler_pred

    print(f"\n{'=' * 60}")
    print(f"=== One-Step ω Prediction Residual (rad/s) ===")
    print(f"{'':6s} {'Pred':>8s} {'IMU':>8s} {'Odom':>8s} {'Res(IMU)':>10s} {'Res(Odom)':>10s}")
    for i, n in enumerate(['wx', 'wy', 'wz']):
        print(f"{n:6s} {np.std(omega_pred[:, i]):8.4f} {np.std(w_imu[:, i]):8.4f} "
              f"{np.std(odom_w_interp[:, i]):8.4f} {np.std(res_w_imu[:, i]):8.4f} "
              f"{np.std(res_w_odom[:, i]):8.4f}")

    print(f"\n=== One-Step Attitude Residual (deg) ===")
    print(f"{'':8s} {'Pred':>8s} {'IMU':>8s} {'Odom':>8s} {'Res(IMU)':>10s} {'Res(Odom)':>10s}")
    for i, n in enumerate(['roll', 'pitch', 'yaw']):
        print(f"{n:8s} {np.std(euler_pred[:, i]):8.3f} {np.std(euler_imu[:, i]):8.3f} "
              f"{np.std(euler_odom[:, i]):8.3f} {np.std(res_att_imu[:, i]):8.4f} "
              f"{np.std(res_att_odom[:, i]):8.4f}")

    # --- Plot ---
    print(f"\n[7] Generating plot...")
    t_plot = t_sim - t_sim[0]

    fig, axes = plt.subplots(5, 3, figsize=(18, 20))
    fig.suptitle(
        'One-Step Angular Velocity & Attitude Prediction\n'
        f'ω_pred(t+dt) = ω_imu(t) + ω̇_model(t)·dt  |  '
        f'CoM=({com[0] * 1000:.1f}, {com[1] * 1000:.1f}) mm',
        fontsize=13,
    )

    names_w = ['ωx (roll)', 'ωy (pitch)', 'ωz (yaw)']
    names_a = ['Roll', 'Pitch', 'Yaw']

    for i in range(3):
        ax = axes[0, i]
        ax.plot(t_plot, w_imu[:, i], 'b-', alpha=0.5, lw=0.5, label='IMU')
        ax.plot(t_plot, odom_w_interp[:, i], 'g-', alpha=0.5, lw=0.5, label='Odom')
        ax.plot(t_plot, omega_pred[:, i], 'r-', alpha=0.5, lw=0.5, label='Model')
        ax.set_title(names_w[i]); ax.set_ylabel('rad/s')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    for i in range(3):
        ax = axes[1, i]
        ax.plot(t_plot, res_w_imu[:, i], 'b-', alpha=0.4, lw=0.5,
                label=f'IMU−Model (std={np.std(res_w_imu[:, i]):.4f})')
        ax.plot(t_plot, res_w_odom[:, i], 'g-', alpha=0.4, lw=0.5,
                label=f'Odom−Model (std={np.std(res_w_odom[:, i]):.4f})')
        ax.axhline(0, color='k', lw=0.5, ls='--')
        ax.set_title(f'{names_w[i]} Residual'); ax.set_ylabel('rad/s')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    for i in range(3):
        ax = axes[2, i]
        ax.plot(t_plot, euler_imu[:, i], 'b-', alpha=0.5, lw=0.5, label='IMU')
        ax.plot(t_plot, euler_odom[:, i], 'g-', alpha=0.5, lw=0.5, label='Odom')
        ax.plot(t_plot, euler_pred[:, i], 'r-', alpha=0.5, lw=0.5, label='Model')
        ax.set_title(f'{names_a[i]} (deg)'); ax.set_ylabel('deg')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    for i in range(3):
        ax = axes[3, i]
        ax.plot(t_plot, res_att_imu[:, i], 'b-', alpha=0.4, lw=0.5,
                label=f'IMU−Model (std={np.std(res_att_imu[:, i]):.3f}°)')
        ax.plot(t_plot, res_att_odom[:, i], 'g-', alpha=0.4, lw=0.5,
                label=f'Odom−Model (std={np.std(res_att_odom[:, i]):.3f}°)')
        ax.axhline(0, color='k', lw=0.5, ls='--')
        ax.set_title(f'{names_a[i]} Residual (deg)'); ax.set_ylabel('deg')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    fs = 1.0 / np.median(np.diff(t_sim))
    for i in range(3):
        ax = axes[4, i]
        for sig, lbl, c in [
            (w_imu[:, i], 'IMU', 'b'),
            (odom_w_interp[:, i], 'Odom', 'g'),
            (omega_pred[:, i], 'Model', 'r'),
            (res_w_imu[:, i], 'Residual', 'k'),
        ]:
            f, p = welch(sig, fs=fs, nperseg=512)
            ax.semilogy(f, p, c + '-', alpha=0.7, label=lbl)
        ax.set_title(f'{names_w[i]} PSD'); ax.set_xlabel('Hz')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = save_dir / "angular_vel_model_vs_imu.png"
    plt.savefig(str(out_path), dpi=150)
    print(f"  Saved: {out_path}")

    print("\n" + "=" * 60)
    print("Done!")
    print("=" * 60)


if __name__ == "__main__":
    main()
