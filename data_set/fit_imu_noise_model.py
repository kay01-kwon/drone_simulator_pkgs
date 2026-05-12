"""
IMU Noise Model Fitting: noise variance as a function of individual rotor thrusts.

Model:
  σ²_j = c₀_j² + α_j² · Σ Fᵢ²

where Fᵢ = CT · RPMᵢ² is the individual rotor thrust.

Reference (one-step prediction for both):
  - Acceleration:
      v_pred(t) = v_odom(t-dt) + [R(t-dt)·[0,0,F/m] + [0,0,-g]] · dt
      a_ref(t) = [v_odom(t) - v_odom(t-dt)] / dt
      f_ref_body(t) = R(t)^T · [a_ref(t) + g·e3]
  - Angular velocity:
      ω_pred(t) = ω_IMU(t-dt) + I⁻¹[τ - ω×Iω - b·ω] · dt

Residuals:
  - δa = a_IMU - f_ref_body
  - δω = ω_IMU - ω_pred
"""

import sqlite3
import struct
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

CDR_BASE = 4
G = 9.81
CT = 1.255e-7
MOMENT_CONST = 0.01569
MASS = 3.188

IXX = 0.06573874618
IYY = 0.06535731789
IZZ = 0.10317211827
I_DIAG = np.array([IXX, IYY, IZZ])

BX, BY, BZ = 0.115, 0.137, 0.184
B_VEC = np.array([BX, BY, BZ])

MOTOR_POS = np.array([
    [0.2295, 0.1325, 0.062],
    [0.0, 0.2650, 0.062],
    [-0.2295, 0.1325, 0.062],
    [-0.2295, -0.1325, 0.062],
    [0.0, -0.2650, 0.062],
    [0.2295, -0.1325, 0.062],
])
REACTION_SIGN = np.array([-1, +1, -1, +1, -1, +1])

COM_OFFSET = np.array([-0.006, 0.0, 0.0])


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
    qx, qy, qz, qw = struct.unpack_from('<4d', data, off); off += 32 + 72
    wx, wy, wz = struct.unpack_from('<3d', data, off); off += 24 + 72
    ax, ay, az = struct.unpack_from('<3d', data, off)
    return sec + nsec * 1e-9, qx, qy, qz, qw, wx, wy, wz, ax, ay, az


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
    return sec + nsec * 1e-9, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz


def parse_rpm(data):
    off = 4
    sec = struct.unpack_from('<I', data, off)[0]; off += 4
    nsec = struct.unpack_from('<I', data, off)[0]; off += 4
    fl = struct.unpack_from('<I', data, off)[0]; off += 4 + fl
    while off % 4:
        off += 1
    vals = struct.unpack_from('<6i', data, off)
    return sec + nsec * 1e-9, vals


def compute_torques(rpms, r_motors):
    """Compute body-frame torque from RPM values."""
    N = len(rpms)
    torques = np.zeros((N, 3))
    for i in range(N):
        tau = np.zeros(3)
        for m in range(6):
            F_m = CT * rpms[i, m] ** 2
            tau += np.cross(r_motors[m], [0, 0, F_m])
            tau[2] += REACTION_SIGN[m] * MOMENT_CONST * F_m
        torques[i] = tau
    return torques


def ang_dyn(omega, tau):
    """Angular acceleration from Euler's equation."""
    return (tau - np.cross(omega, I_DIAG * omega) - B_VEC * omega) / I_DIAG


def main():
    bag_dir = Path("data_set/2026_05_05_free_flight")
    bag_path = str(bag_dir / "02_ct_1p255" / "02_ct_1p255_0.db3")
    save_dir = Path("data_set/trained_models")
    save_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("IMU Noise Model Fitting (One-Step Prediction)")
    print("σ² = c₀² + α² · Σ Fᵢ²")
    print("=" * 60)

    # ---- Parse all data ----
    print("\n[1] Parsing data...")

    imu_ts, imu_q, imu_w, imu_acc = [], [], [], []
    for _, data in read_bag_topic(bag_path, "/mavros/imu/data"):
        try:
            t, qx, qy, qz, qw, wx, wy, wz, ax, ay, az = parse_imu(data)
            imu_ts.append(t)
            imu_q.append([qx, qy, qz, qw])
            imu_w.append([wx, wy, wz])
            imu_acc.append([ax, ay, az])
        except:
            continue
    imu_ts = np.array(imu_ts)
    imu_q = np.array(imu_q)
    imu_w = np.array(imu_w)
    imu_acc = np.array(imu_acc)
    print(f"  IMU: {len(imu_ts)} samples")

    od_ts, od_vel = [], []
    for _, data in read_bag_topic(bag_path, "/mavros/local_position/odom"):
        try:
            t, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz = parse_odom(data)
            od_ts.append(t)
            od_vel.append([vx, vy, vz])
        except:
            continue
    od_ts = np.array(od_ts)
    od_vel = np.array(od_vel)
    print(f"  Odom: {len(od_ts)} samples")

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

    # ---- Time alignment ----
    t0 = min(imu_ts[0], od_ts[0], rpm_ts[0])
    imu_ts -= t0
    od_ts -= t0
    rpm_ts -= t0

    t_start = 1.0
    t_end = min(imu_ts[-1], od_ts[-1], rpm_ts[-1]) - 1.0

    # ---- Interpolate odom velocity & RPM to IMU time grid ----
    mask = (imu_ts >= t_start) & (imu_ts <= t_end)
    t_c = imu_ts[mask]
    N = len(t_c)
    print(f"\n[2] Analysis window: {t_start:.1f}-{t_end:.1f}s, N={N}")

    imu_acc_c = imu_acc[mask]
    imu_w_c = imu_w[mask]
    imu_q_c = imu_q[mask]

    od_vel_interp = np.zeros((N, 3))
    for ax in range(3):
        f = interp1d(od_ts, od_vel[:, ax], kind='linear', fill_value='extrapolate')
        od_vel_interp[:, ax] = f(t_c)

    rpm_interp = np.zeros((N, 6))
    for m in range(6):
        f = interp1d(rpm_ts, rpm_vals[:, m], kind='previous', fill_value='extrapolate')
        rpm_interp[:, m] = f(t_c)

    # ---- Compute motor torques ----
    print("[3] Computing motor torques...")
    r_motors = MOTOR_POS - COM_OFFSET
    torques = compute_torques(rpm_interp, r_motors)

    # ---- One-step prediction: acceleration (translational) ----
    print("[4] One-step acceleration prediction...")
    # v_pred(t) = v_odom(t-dt) + [R(t-dt)·[0,0,F(t-dt)/m] + [0,0,-g]] · dt
    # a_onestep(t) = [v_odom(t) - v_odom(t-dt)] / dt   (actual from odom)
    # f_ref_body(t) = R(t)^T · [a_onestep + g·e3]

    g_vec = np.array([0.0, 0.0, G])
    delta_a = np.zeros((N, 3))
    valid_a = np.ones(N, dtype=bool)

    for i in range(1, N):
        dt = t_c[i] - t_c[i - 1]
        if dt <= 0 or dt > 0.05:
            valid_a[i] = False
            continue

        # Actual inertial acceleration from odom finite difference
        a_odom = (od_vel_interp[i] - od_vel_interp[i - 1]) / dt

        # Body-frame specific force reference
        R_curr = Rotation.from_quat(imu_q_c[i]).as_matrix()
        f_ref_body = R_curr.T @ (a_odom + g_vec)

        # Residual: IMU - reference
        delta_a[i] = imu_acc_c[i] - f_ref_body

    valid_a[0] = False

    # ---- One-step prediction: angular velocity ----
    print("[5] One-step angular velocity prediction...")
    delta_w = np.zeros((N, 3))
    valid_w = np.ones(N, dtype=bool)

    for i in range(1, N):
        dt = t_c[i] - t_c[i - 1]
        if dt <= 0 or dt > 0.05:
            valid_w[i] = False
            continue

        wdot = ang_dyn(imu_w_c[i - 1], torques[i - 1])
        w_pred = imu_w_c[i - 1] + wdot * dt
        delta_w[i] = imu_w_c[i] - w_pred

    valid_w[0] = False

    # ---- Use only valid samples ----
    valid = valid_a & valid_w
    t_v = t_c[valid]
    delta_a_v = delta_a[valid]
    delta_w_v = delta_w[valid]
    rpm_v = rpm_interp[valid]
    N_v = len(t_v)
    print(f"  Valid samples: {N_v}/{N}")

    # ---- Compute per-motor thrust and ΣFi² ----
    Fi_v = CT * rpm_v ** 2
    sum_Fi2_v = np.sum(Fi_v ** 2, axis=1)

    # ---- Windowed variance estimation ----
    print("\n[6] Fitting noise model (windowed variance)...")
    win_samples = 100
    n_windows = N_v // win_samples

    win_var_a = np.zeros((n_windows, 3))
    win_var_w = np.zeros((n_windows, 3))
    win_sum_Fi2 = np.zeros(n_windows)

    for k in range(n_windows):
        s = k * win_samples
        e = s + win_samples
        for j in range(3):
            win_var_a[k, j] = np.var(delta_a_v[s:e, j])
            win_var_w[k, j] = np.var(delta_w_v[s:e, j])
        win_sum_Fi2[k] = np.mean(sum_Fi2_v[s:e])

    # ---- Linear regression: var = c0² + α² · ΣFi² ----
    A = np.column_stack([np.ones(n_windows), win_sum_Fi2])

    print(f"\n{'=' * 60}")
    print(f"Noise Model: σ² = c₀² + α² · Σ Fᵢ²")
    print(f"{'=' * 60}")

    params_a = np.zeros((3, 2))
    params_w = np.zeros((3, 2))

    a_labels = ['ax', 'ay', 'az']
    w_labels = ['ωx', 'ωy', 'ωz']

    print(f"\n--- Accelerometer (ref: odom one-step) ---")
    print(f"{'Axis':>6s} {'c₀ (m/s²)':>12s} {'α (m/s²/N)':>14s} {'R²':>8s}")
    for j in range(3):
        y = win_var_a[:, j]
        coef, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        params_a[j] = coef
        c0 = np.sqrt(max(coef[0], 0))
        alpha = np.sqrt(max(coef[1], 0))
        y_pred = A @ coef
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
        print(f"{a_labels[j]:>6s} {c0:12.4f} {alpha:14.4f} {r2:8.4f}")

    print(f"\n--- Gyroscope (ref: torque model one-step) ---")
    print(f"{'Axis':>6s} {'c₀ (rad/s)':>12s} {'α (rad/s/N)':>14s} {'R²':>8s}")
    for j in range(3):
        y = win_var_w[:, j]
        coef, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
        params_w[j] = coef
        c0 = np.sqrt(max(coef[0], 0))
        alpha = np.sqrt(max(coef[1], 0))
        y_pred = A @ coef
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
        print(f"{w_labels[j]:>6s} {c0:12.6f} {alpha:14.6f} {r2:8.4f}")

    # ---- Summary at hover ----
    hover_rpm = np.sqrt(MASS * G / (6 * CT))
    hover_Fi = CT * hover_rpm ** 2
    hover_sum_Fi2 = 6 * hover_Fi ** 2

    print(f"\n--- Hover operating point ---")
    print(f"  RPM_hover = {hover_rpm:.0f}")
    print(f"  F_per_motor = {hover_Fi:.3f} N")
    print(f"  Σ Fᵢ² = {hover_sum_Fi2:.4f} N²")

    print(f"\n--- Predicted noise std at hover ---")
    print(f"{'Axis':>6s} {'σ_total':>12s} {'σ_c0':>12s} {'σ_motor':>12s}")
    for j in range(3):
        var_total = max(params_a[j, 0], 0) + max(params_a[j, 1], 0) * hover_sum_Fi2
        sigma_t = np.sqrt(var_total)
        sigma_c0 = np.sqrt(max(params_a[j, 0], 0))
        sigma_m = np.sqrt(max(params_a[j, 1], 0) * hover_sum_Fi2)
        print(f"{a_labels[j]:>6s} {sigma_t:12.4f} {sigma_c0:12.4f} {sigma_m:12.4f} m/s²")
    for j in range(3):
        var_total = max(params_w[j, 0], 0) + max(params_w[j, 1], 0) * hover_sum_Fi2
        sigma_t = np.sqrt(var_total)
        sigma_c0 = np.sqrt(max(params_w[j, 0], 0))
        sigma_m = np.sqrt(max(params_w[j, 1], 0) * hover_sum_Fi2)
        print(f"{w_labels[j]:>6s} {sigma_t:12.6f} {sigma_c0:12.6f} {sigma_m:12.6f} rad/s")

    # ---- Overall std (for quick reference) ----
    print(f"\n--- Overall residual std ---")
    for j in range(3):
        print(f"  {a_labels[j]}: {np.std(delta_a_v[:, j]):.4f} m/s²")
    for j in range(3):
        print(f"  {w_labels[j]}: {np.std(delta_w_v[:, j]):.6f} rad/s")

    # ---- Save model ----
    model_path = save_dir / "imu_noise_model.npz"
    np.savez(str(model_path),
             params_a=params_a,
             params_w=params_w,
             CT=CT,
             hover_rpm=hover_rpm,
             com_offset=COM_OFFSET,
             description="var = params[:,0] + params[:,1] * sum_Fi2; one-step prediction ref")
    print(f"\n  Model saved: {model_path}")

    # ---- Plot ----
    print("\n[7] Generating plots...")
    # Use full arrays with valid mask for plotting
    sum_Fi2_full = np.sum((CT * rpm_interp ** 2) ** 2, axis=1)

    fig, axes = plt.subplots(4, 3, figsize=(18, 16))
    fig.suptitle(
        'IMU Noise Model (One-Step Prediction): σ² = c₀² + α² · Σ Fᵢ²\n'
        'Accel ref: R^T·(Δv_odom/dt + g·e3)  |  Gyro ref: ω + I⁻¹[τ−ω×Iω−bω]·dt',
        fontsize=13
    )

    # Row 0: variance vs ΣFi² (accelerometer)
    for j in range(3):
        ax = axes[0, j]
        ax.scatter(win_sum_Fi2, win_var_a[:, j], s=3, alpha=0.4, c='b', label='Data')
        x_fit = np.linspace(0, win_sum_Fi2.max() * 1.05, 100)
        y_fit = np.maximum(params_a[j, 0], 0) + np.maximum(params_a[j, 1], 0) * x_fit
        ax.plot(x_fit, y_fit, 'r-', lw=2, label='Fit')
        c0 = np.sqrt(max(params_a[j, 0], 0))
        alpha = np.sqrt(max(params_a[j, 1], 0))
        ax.set_title(f'{a_labels[j]}: c₀={c0:.3f}, α={alpha:.3f}')
        ax.set_xlabel('Σ Fᵢ² (N²)')
        ax.set_ylabel('Var (m²/s⁴)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # Row 1: variance vs ΣFi² (gyroscope)
    for j in range(3):
        ax = axes[1, j]
        ax.scatter(win_sum_Fi2, win_var_w[:, j], s=3, alpha=0.4, c='b', label='Data')
        x_fit = np.linspace(0, win_sum_Fi2.max() * 1.05, 100)
        y_fit = np.maximum(params_w[j, 0], 0) + np.maximum(params_w[j, 1], 0) * x_fit
        ax.plot(x_fit, y_fit, 'r-', lw=2, label='Fit')
        c0 = np.sqrt(max(params_w[j, 0], 0))
        alpha = np.sqrt(max(params_w[j, 1], 0))
        ax.set_title(f'{w_labels[j]}: c₀={c0:.6f}, α={alpha:.6f}')
        ax.set_xlabel('Σ Fᵢ² (N²)')
        ax.set_ylabel('Var (rad²/s²)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # Row 2: residual time series (accel)
    t_plot = t_v - t_v[0]
    for j in range(3):
        ax = axes[2, j]
        ax.plot(t_plot, delta_a_v[:, j], 'b-', alpha=0.3, lw=0.3)
        sigma_pred = np.sqrt(
            np.maximum(params_a[j, 0], 0)
            + np.maximum(params_a[j, 1], 0) * sum_Fi2_v
        )
        ax.plot(t_plot, sigma_pred, 'r-', lw=1, label='+σ')
        ax.plot(t_plot, -sigma_pred, 'r-', lw=1, label='−σ')
        ax.set_title(f'δ{a_labels[j]} residual')
        ax.set_ylabel('m/s²')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 3: residual time series (gyro)
    for j in range(3):
        ax = axes[3, j]
        ax.plot(t_plot, delta_w_v[:, j], 'b-', alpha=0.3, lw=0.3)
        sigma_pred = np.sqrt(
            np.maximum(params_w[j, 0], 0)
            + np.maximum(params_w[j, 1], 0) * sum_Fi2_v
        )
        ax.plot(t_plot, sigma_pred, 'r-', lw=1, label='+σ')
        ax.plot(t_plot, -sigma_pred, 'r-', lw=1, label='−σ')
        ax.set_title(f'δ{w_labels[j]} residual')
        ax.set_ylabel('rad/s')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Time (s)')

    plt.tight_layout()
    out_path = save_dir / "imu_noise_model_fit.png"
    plt.savefig(str(out_path), dpi=150)
    print(f"  Saved: {out_path}")

    # ---- Histogram of normalized residuals ----
    fig2, axes2 = plt.subplots(2, 3, figsize=(16, 8))
    fig2.suptitle('Normalized Residuals: δ / σ_model (should be ≈ N(0,1))', fontsize=13)

    for j in range(3):
        sigma = np.sqrt(
            np.maximum(params_a[j, 0], 0)
            + np.maximum(params_a[j, 1], 0) * sum_Fi2_v
        )
        normalized = delta_a_v[:, j] / np.maximum(sigma, 1e-10)
        ax = axes2[0, j]
        ax.hist(normalized, bins=200, density=True, alpha=0.7, color='b')
        x_g = np.linspace(-5, 5, 200)
        ax.plot(x_g, np.exp(-x_g ** 2 / 2) / np.sqrt(2 * np.pi), 'r-', lw=2)
        ax.set_title(f'{a_labels[j]}: std={np.std(normalized):.3f}')
        ax.set_xlim(-6, 6)
        ax.grid(True, alpha=0.3)

    for j in range(3):
        sigma = np.sqrt(
            np.maximum(params_w[j, 0], 0)
            + np.maximum(params_w[j, 1], 0) * sum_Fi2_v
        )
        normalized = delta_w_v[:, j] / np.maximum(sigma, 1e-10)
        ax = axes2[1, j]
        ax.hist(normalized, bins=200, density=True, alpha=0.7, color='b')
        x_g = np.linspace(-5, 5, 200)
        ax.plot(x_g, np.exp(-x_g ** 2 / 2) / np.sqrt(2 * np.pi), 'r-', lw=2)
        ax.set_title(f'{w_labels[j]}: std={np.std(normalized):.3f}')
        ax.set_xlim(-6, 6)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out2 = save_dir / "imu_noise_normalized_hist.png"
    plt.savefig(str(out2), dpi=150)
    print(f"  Saved: {out2}")

    print(f"\n{'=' * 60}")
    print("Done!")


if __name__ == "__main__":
    main()
