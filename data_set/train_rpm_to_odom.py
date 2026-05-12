"""
Direct RPM → Odom state transition learning.

One-step prediction:
  state_pred(t+dt) = dynamics_model(state_odom(t), RPM(t))
  residual = state_odom(t+dt) - state_pred(t+dt)
  GPR learns: residual = f(RPM, state)

This captures model errors + EKF2 filtering + IMU noise in one shot.
"""

import sqlite3
import struct
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel
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
    [0.2295, 0.1325, 0.062], [0.0, 0.2650, 0.062], [-0.2295, 0.1325, 0.062],
    [-0.2295, -0.1325, 0.062], [0.0, -0.2650, 0.062], [0.2295, -0.1325, 0.062],
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


def parse_rpm(data):
    off = 4
    sec = struct.unpack_from('<I', data, off)[0]; off += 4
    nsec = struct.unpack_from('<I', data, off)[0]; off += 4
    fl = struct.unpack_from('<I', data, off)[0]; off += 4 + fl
    while off % 4:
        off += 1
    vals = struct.unpack_from('<6i', data, off)
    return sec + nsec * 1e-9, vals


def dynamics_vel_prediction(v, q, rpm, dt):
    """One-step velocity prediction (world frame)."""
    R = Rotation.from_quat(q).as_matrix()
    F_total = CT * np.sum(rpm ** 2)
    F_body = np.array([0.0, 0.0, F_total])
    a_inertial = R @ F_body / MASS + np.array([0.0, 0.0, -G])
    return v + a_inertial * dt


def dynamics_omega_prediction(omega, rpm, dt):
    """One-step angular velocity prediction (body frame)."""
    r_motors = MOTOR_POS - COM_OFFSET
    tau = np.zeros(3)
    for m in range(6):
        F_m = CT * rpm[m] ** 2
        tau += np.cross(r_motors[m], [0, 0, F_m])
        tau[2] += REACTION_SIGN[m] * MOMENT_CONST * F_m
    wdot = (tau - np.cross(omega, I_DIAG * omega) - B_VEC * omega) / I_DIAG
    return omega + wdot * dt


def main():
    bag_dir = Path("data_set/2026_05_05_free_flight")
    bag_path = str(bag_dir / "02_ct_1p255" / "02_ct_1p255_0.db3")
    save_dir = Path("data_set/trained_models")
    save_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("RPM → Odom State Transition Learning")
    print("=" * 60)

    # ---- Parse data ----
    print("\n[1] Parsing data...")

    od_ts, od_pos, od_q, od_vel, od_w = [], [], [], [], []
    for _, data in read_bag_topic(bag_path, "/mavros/local_position/odom"):
        try:
            t, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz = parse_odom(data)
            od_ts.append(t)
            od_pos.append([px, py, pz])
            od_q.append([qx, qy, qz, qw])
            od_vel.append([vx, vy, vz])
            od_w.append([wx, wy, wz])
        except:
            continue
    od_ts = np.array(od_ts)
    od_pos = np.array(od_pos)
    od_q = np.array(od_q)
    od_vel = np.array(od_vel)
    od_w = np.array(od_w)
    print(f"  Odom: {len(od_ts)} samples, dt_med={np.median(np.diff(od_ts))*1000:.1f}ms")

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
    t0 = min(od_ts[0], rpm_ts[0])
    od_ts -= t0
    rpm_ts -= t0

    # Interpolate RPM to odom timestamps
    rpm_interp = np.zeros((len(od_ts), 6))
    for m in range(6):
        f = interp1d(rpm_ts, rpm_vals[:, m], kind='previous', fill_value='extrapolate')
        rpm_interp[:, m] = f(od_ts)

    # ---- Compute one-step residuals ----
    print("\n[2] Computing one-step residuals...")
    N = len(od_ts)

    res_vel = np.zeros((N, 3))   # velocity residual (world frame)
    res_w = np.zeros((N, 3))     # angular velocity residual (body frame)
    valid = np.zeros(N, dtype=bool)

    for i in range(1, N):
        dt = od_ts[i] - od_ts[i - 1]
        if dt <= 0 or dt > 0.05:
            continue

        # Velocity prediction
        v_pred = dynamics_vel_prediction(od_vel[i - 1], od_q[i - 1], rpm_interp[i - 1], dt)
        res_vel[i] = od_vel[i] - v_pred

        # Angular velocity prediction
        w_pred = dynamics_omega_prediction(od_w[i - 1], rpm_interp[i - 1], dt)
        res_w[i] = od_w[i] - w_pred

        valid[i] = True

    print(f"  Valid: {valid.sum()}/{N}")

    # ---- Prepare features and targets ----
    t_v = od_ts[valid]
    rpm_v = rpm_interp[valid]
    vel_v = od_vel[valid]
    w_v = od_w[valid]
    q_v = od_q[valid]
    res_vel_v = res_vel[valid]
    res_w_v = res_w[valid]

    Fi_v = CT * rpm_v ** 2  # individual thrusts (N, 6)

    # Features: 6 motor thrusts
    X = Fi_v.copy()

    print(f"\n[3] Feature: individual motor thrusts F_i (6 dims)")
    print(f"  X shape: {X.shape}")

    # ---- Residual statistics ----
    print(f"\n{'='*60}")
    print("One-step residual statistics (odom_actual - model_pred)")
    print(f"{'='*60}")
    v_labels = ['vx', 'vy', 'vz']
    w_labels = ['ωx', 'ωy', 'ωz']
    print(f"\n--- Velocity residual (m/s) ---")
    for j in range(3):
        print(f"  {v_labels[j]}: mean={np.mean(res_vel_v[:,j]):.6f}, std={np.std(res_vel_v[:,j]):.6f}")
    print(f"\n--- Angular velocity residual (rad/s) ---")
    for j in range(3):
        print(f"  {w_labels[j]}: mean={np.mean(res_w_v[:,j]):.6f}, std={np.std(res_w_v[:,j]):.6f}")

    # ---- Train GPR (sparse, subsample for speed) ----
    print(f"\n[4] Training Sparse GPR...")
    n_train = min(800, len(X))
    rng = np.random.RandomState(42)
    idx_train = rng.choice(len(X), n_train, replace=False)
    idx_train.sort()
    X_train = X[idx_train]

    # Normalize features
    X_mean = X_train.mean(axis=0)
    X_std = X_train.std(axis=0)
    X_std[X_std < 1e-10] = 1.0
    X_train_n = (X_train - X_mean) / X_std
    X_all_n = (X - X_mean) / X_std

    gpr_models = {}
    all_labels = v_labels + w_labels
    all_res = np.hstack([res_vel_v, res_w_v])

    for j, label in enumerate(all_labels):
        print(f"  Training {label}...", end=" ", flush=True)
        y_train = all_res[idx_train, j]

        kernel = ConstantKernel(1.0) * RBF(length_scale=np.ones(6)) + WhiteKernel(noise_level=1.0)
        gpr = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=3, random_state=42)
        gpr.fit(X_train_n, y_train)
        y_pred, y_std = gpr.predict(X_all_n, return_std=True)

        rmse = np.sqrt(np.mean((all_res[:, j] - y_pred) ** 2))
        print(f"RMSE={rmse:.6f}, kernel: {gpr.kernel_}")

        gpr_models[label] = {
            'gpr': gpr,
            'y_pred': y_pred,
            'y_std': y_std,
            'rmse': rmse,
        }

    # ---- Plot ----
    print(f"\n[5] Generating plots...")
    t_plot = t_v - t_v[0]

    fig, axes = plt.subplots(4, 3, figsize=(18, 16))
    fig.suptitle(
        'RPM → Odom: One-Step Residual & GPR Fit\n'
        'residual = odom(t+dt) - dynamics_model(odom(t), RPM(t))',
        fontsize=13,
    )

    # Row 0: velocity residual time series
    for j in range(3):
        ax = axes[0, j]
        m = gpr_models[v_labels[j]]
        ax.plot(t_plot, res_vel_v[:, j], 'b-', alpha=0.3, lw=0.3, label='Actual residual')
        ax.plot(t_plot, m['y_pred'], 'r-', alpha=0.5, lw=0.5, label=f'GPR (RMSE={m["rmse"]:.6f})')
        ax.fill_between(t_plot, m['y_pred'] - 2*m['y_std'], m['y_pred'] + 2*m['y_std'],
                        alpha=0.1, color='r')
        ax.set_title(f'{v_labels[j]} residual (m/s)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 1: angular velocity residual time series
    for j in range(3):
        ax = axes[1, j]
        m = gpr_models[w_labels[j]]
        ax.plot(t_plot, res_w_v[:, j], 'b-', alpha=0.3, lw=0.3, label='Actual residual')
        ax.plot(t_plot, m['y_pred'], 'r-', alpha=0.5, lw=0.5, label=f'GPR (RMSE={m["rmse"]:.6f})')
        ax.fill_between(t_plot, m['y_pred'] - 2*m['y_std'], m['y_pred'] + 2*m['y_std'],
                        alpha=0.1, color='r')
        ax.set_title(f'{w_labels[j]} residual (rad/s)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 2: residual histograms (velocity)
    for j in range(3):
        ax = axes[2, j]
        r = res_vel_v[:, j]
        ax.hist(r, bins=100, density=True, alpha=0.7, color='b', label='Residual')
        x_g = np.linspace(r.min(), r.max(), 200)
        mu, sig = np.mean(r), np.std(r)
        ax.plot(x_g, np.exp(-(x_g - mu)**2 / (2*sig**2)) / (sig*np.sqrt(2*np.pi)),
                'r-', lw=2, label=f'N({mu:.4f},{sig:.4f}²)')
        ax.set_title(f'{v_labels[j]} histogram')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 3: residual histograms (angular velocity)
    for j in range(3):
        ax = axes[3, j]
        r = res_w_v[:, j]
        ax.hist(r, bins=100, density=True, alpha=0.7, color='b', label='Residual')
        x_g = np.linspace(r.min(), r.max(), 200)
        mu, sig = np.mean(r), np.std(r)
        ax.plot(x_g, np.exp(-(x_g - mu)**2 / (2*sig**2)) / (sig*np.sqrt(2*np.pi)),
                'r-', lw=2, label=f'N({mu:.5f},{sig:.5f}²)')
        ax.set_title(f'{w_labels[j]} histogram')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out1 = save_dir / "rpm_to_odom_residual.png"
    plt.savefig(str(out1), dpi=150)
    print(f"  Saved: {out1}")

    # ---- Open-loop simulation ----
    print(f"\n[6] Open-loop simulation (no reset)...")
    vel_sim = np.zeros((len(t_v), 3))
    w_sim = np.zeros((len(t_v), 3))
    vel_sim[0] = vel_v[0]
    w_sim[0] = w_v[0]

    q_sim = np.zeros((len(t_v), 4))
    q_sim[0] = q_v[0]

    for i in range(1, len(t_v)):
        dt = t_v[i] - t_v[i - 1]
        if dt <= 0 or dt > 0.05:
            vel_sim[i] = vel_v[i]
            w_sim[i] = w_v[i]
            q_sim[i] = q_v[i]
            continue

        # Velocity: model + GPR correction
        v_model = dynamics_vel_prediction(vel_sim[i - 1], q_sim[i - 1], rpm_v[i - 1], dt)
        w_model = dynamics_omega_prediction(w_sim[i - 1], rpm_v[i - 1], dt)

        x_i = ((Fi_v[i - 1:i]) - X_mean) / X_std
        gpr_corr_v = np.array([gpr_models[l]['gpr'].predict(x_i)[0] for l in v_labels])
        gpr_corr_w = np.array([gpr_models[l]['gpr'].predict(x_i)[0] for l in w_labels])

        vel_sim[i] = v_model + gpr_corr_v
        w_sim[i] = w_model + gpr_corr_w

        # Simple quaternion integration for attitude
        omega = w_sim[i]
        wnorm = np.linalg.norm(omega)
        if wnorm > 1e-10:
            ha = 0.5 * wnorm * dt
            s = np.sin(ha) / wnorm
            dq = np.array([s*omega[0], s*omega[1], s*omega[2], np.cos(ha)])
            x1, y1, z1, w1 = q_sim[i - 1]
            x2, y2, z2, w2 = dq
            q_sim[i] = np.array([
                w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2,
            ])
            q_sim[i] /= np.linalg.norm(q_sim[i])
        else:
            q_sim[i] = q_sim[i - 1]

    # Plot open-loop
    fig2, axes2 = plt.subplots(2, 3, figsize=(18, 8))
    fig2.suptitle('Open-Loop Simulation: Model + GPR vs Actual Odom', fontsize=13)

    for j in range(3):
        ax = axes2[0, j]
        ax.plot(t_plot, vel_v[:, j], 'b-', alpha=0.5, lw=0.5, label='Odom actual')
        ax.plot(t_plot, vel_sim[:, j], 'r-', alpha=0.5, lw=0.5, label='Model+GPR')
        ax.set_title(f'{v_labels[j]} (m/s)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    for j in range(3):
        ax = axes2[1, j]
        ax.plot(t_plot, w_v[:, j], 'b-', alpha=0.5, lw=0.5, label='Odom actual')
        ax.plot(t_plot, w_sim[:, j], 'r-', alpha=0.5, lw=0.5, label='Model+GPR')
        ax.set_title(f'{w_labels[j]} (rad/s)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Time (s)')

    plt.tight_layout()
    out2 = save_dir / "rpm_to_odom_openloop.png"
    plt.savefig(str(out2), dpi=150)
    print(f"  Saved: {out2}")

    print(f"\n--- Open-loop velocity error ---")
    for j in range(3):
        err = vel_v[:, j] - vel_sim[:, j]
        print(f"  {v_labels[j]}: RMSE={np.sqrt(np.mean(err**2)):.4f} m/s")
    print(f"\n--- Open-loop angular velocity error ---")
    for j in range(3):
        err = w_v[:, j] - w_sim[:, j]
        print(f"  {w_labels[j]}: RMSE={np.sqrt(np.mean(err**2)):.4f} rad/s")

    print(f"\n{'='*60}")
    print("Done!")


if __name__ == "__main__":
    main()
