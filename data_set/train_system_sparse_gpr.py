"""
System Dynamics Residual Learning with Sparse GPR
==================================================
Learns the residual between rigid-body dynamics prediction and actual
acceleration from odom, during hovering flight.

Input: [Fz, tau_x, tau_y, tau_z, roll, pitch, vx, vy, vz]
Output: [ax, ay, az] residual (actual - predicted)

Uses GPyTorch Variational SGPR with inducing points.
"""

import struct
import sqlite3
import numpy as np
import torch
import gpytorch
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# ============================================================
# S550 Parameters
# ============================================================

MASS = 3.188  # kg
G = 9.81      # m/s^2
CT = 1.255e-7 # N/(rpm)^2
IXX = 0.06573874618
IYY = 0.06535731789
IZZ = 0.10317211827

HOVER_Z_THRESHOLD = 0.05  # m relative height to consider hovering


# ============================================================
# 1. Data extraction from rosbag
# ============================================================

def extract_all_data(bag_dir: str):
    """Extract odom, control, and IMU data from rosbag."""
    bag_path = Path(bag_dir)
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    odom_data = {"ts": [], "px": [], "py": [], "pz": [],
                 "qx": [], "qy": [], "qz": [], "qw": [],
                 "vx": [], "vy": [], "vz": [],
                 "wx": [], "wy": [], "wz": []}
    ctrl_data = {"ts": [], "fz": [], "tx": [], "ty": [], "tz": []}
    imu_data = {"ts": [], "ax": [], "ay": [], "az": [],
                "wx": [], "wy": [], "wz": []}

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == "/mavros/local_position/odom":
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                odom_data["ts"].append(t)
                odom_data["px"].append(msg.pose.pose.position.x)
                odom_data["py"].append(msg.pose.pose.position.y)
                odom_data["pz"].append(msg.pose.pose.position.z)
                odom_data["qx"].append(msg.pose.pose.orientation.x)
                odom_data["qy"].append(msg.pose.pose.orientation.y)
                odom_data["qz"].append(msg.pose.pose.orientation.z)
                odom_data["qw"].append(msg.pose.pose.orientation.w)
                odom_data["vx"].append(msg.twist.twist.linear.x)
                odom_data["vy"].append(msg.twist.twist.linear.y)
                odom_data["vz"].append(msg.twist.twist.linear.z)
                odom_data["wx"].append(msg.twist.twist.angular.x)
                odom_data["wy"].append(msg.twist.twist.angular.y)
                odom_data["wz"].append(msg.twist.twist.angular.z)

            elif connection.topic == "/nmpc/control":
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                ctrl_data["ts"].append(t)
                ctrl_data["fz"].append(msg.wrench.force.z)
                ctrl_data["tx"].append(msg.wrench.torque.x)
                ctrl_data["ty"].append(msg.wrench.torque.y)
                ctrl_data["tz"].append(msg.wrench.torque.z)

            elif connection.topic == "/mavros/imu/data":
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                imu_data["ts"].append(t)
                imu_data["ax"].append(msg.linear_acceleration.x)
                imu_data["ay"].append(msg.linear_acceleration.y)
                imu_data["az"].append(msg.linear_acceleration.z)
                imu_data["wx"].append(msg.angular_velocity.x)
                imu_data["wy"].append(msg.angular_velocity.y)
                imu_data["wz"].append(msg.angular_velocity.z)

    for key in odom_data:
        odom_data[key] = np.array(odom_data[key])
    for key in ctrl_data:
        ctrl_data[key] = np.array(ctrl_data[key])
    for key in imu_data:
        imu_data[key] = np.array(imu_data[key])

    # Parse actual RPM from sqlite
    db_files = list(bag_path.glob("*.db3"))
    conn = sqlite3.connect(str(db_files[0]))
    topics = conn.execute("SELECT id, name FROM topics").fetchall()
    topic_map = {t[1]: t[0] for t in topics}
    rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_map["/uav/actual_rpm"],),
    ).fetchall()

    rpm_data = {"ts": [], "vals": []}
    for _, data_bytes in rows:
        offset = 4
        sec, nsec = struct.unpack_from("<II", data_bytes, offset)
        offset += 8
        frame_len = struct.unpack_from("<I", data_bytes, offset)[0]
        offset += 4 + frame_len
        while offset % 4:
            offset += 1
        vals = struct.unpack_from("<6i", data_bytes, offset)
        rpm_data["ts"].append(sec + nsec * 1e-9)
        rpm_data["vals"].append(list(vals))
    conn.close()

    rpm_data["ts"] = np.array(rpm_data["ts"])
    rpm_data["vals"] = np.array(rpm_data["vals"], dtype=np.float64)

    return odom_data, ctrl_data, imu_data, rpm_data


def compute_euler_from_quat(qx, qy, qz, qw):
    """Convert quaternion arrays to Euler angles (roll, pitch, yaw)."""
    quats = np.stack([qx, qy, qz, qw], axis=1)
    rot = Rotation.from_quat(quats)
    euler = rot.as_euler("xyz", degrees=False)
    return euler[:, 0], euler[:, 1], euler[:, 2]


def compute_accel_from_velocity(ts, vx, vy, vz):
    """Compute acceleration from velocity using central differences."""
    N = len(ts)
    ax = np.zeros(N)
    ay = np.zeros(N)
    az = np.zeros(N)

    for i in range(1, N - 1):
        dt = ts[i + 1] - ts[i - 1]
        if dt > 0:
            ax[i] = (vx[i + 1] - vx[i - 1]) / dt
            ay[i] = (vy[i + 1] - vy[i - 1]) / dt
            az[i] = (vz[i + 1] - vz[i - 1]) / dt

    ax[0] = ax[1]
    ay[0] = ay[1]
    az[0] = az[1]
    ax[-1] = ax[-2]
    ay[-1] = ay[-2]
    az[-1] = az[-2]

    return ax, ay, az


def predict_rigid_body_accel(fz, roll, pitch):
    """
    Predict world-frame linear acceleration from thrust and orientation.
    Thrust is along body z-axis, rotated to world frame.

    a_x = -(Fz/m) * sin(pitch)
    a_y =  (Fz/m) * sin(roll) * cos(pitch)
    a_z =  (Fz/m) * cos(roll) * cos(pitch) - g
    """
    ax_pred = -(fz / MASS) * np.sin(pitch)
    ay_pred = (fz / MASS) * np.sin(roll) * np.cos(pitch)
    az_pred = (fz / MASS) * np.cos(roll) * np.cos(pitch) - G
    return ax_pred, ay_pred, az_pred


# ============================================================
# 2. Sparse GPR Model (GPyTorch Variational SGPR)
# ============================================================

class SparseGPModel(gpytorch.models.ApproximateGP):
    def __init__(self, inducing_points):
        variational_distribution = gpytorch.variational.CholeskyVariationalDistribution(
            inducing_points.size(0)
        )
        variational_strategy = gpytorch.variational.VariationalStrategy(
            self, inducing_points, variational_distribution,
            learn_inducing_locations=True,
        )
        super().__init__(variational_strategy)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(
            gpytorch.kernels.RBFKernel(ard_num_dims=inducing_points.size(1))
        )

    def forward(self, x):
        mean = self.mean_module(x)
        covar = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean, covar)


def train_sgpr(train_x, train_y, n_inducing=200, n_epochs=500, lr=0.01):
    """Train a variational SGPR model."""
    x_mean = train_x.mean(dim=0)
    x_std = train_x.std(dim=0)
    x_std[x_std < 1e-6] = 1.0
    train_x_norm = (train_x - x_mean) / x_std

    y_mean = train_y.mean()
    y_std = train_y.std()
    if y_std < 1e-6:
        y_std = torch.tensor(1.0)
    train_y_norm = (train_y - y_mean) / y_std

    n_inducing = min(n_inducing, len(train_x) // 5)
    inducing_idx = torch.randperm(train_x_norm.size(0))[:n_inducing]
    inducing_points = train_x_norm[inducing_idx].clone()

    likelihood = gpytorch.likelihoods.GaussianLikelihood()
    model = SparseGPModel(inducing_points)

    model.train()
    likelihood.train()

    optimizer = torch.optim.Adam(
        list(model.parameters()) + list(likelihood.parameters()), lr=lr
    )
    mll = gpytorch.mlls.VariationalELBO(likelihood, model, num_data=train_x_norm.size(0))

    losses = []
    for epoch in range(n_epochs):
        optimizer.zero_grad()
        output = model(train_x_norm)
        loss = -mll(output, train_y_norm)
        loss.backward()
        optimizer.step()
        losses.append(loss.item())
        if (epoch + 1) % 100 == 0:
            print(f"      Epoch {epoch+1}/{n_epochs}, Loss: {loss.item():.4f}")

    model.eval()
    likelihood.eval()

    return model, likelihood, x_mean, x_std, y_mean, y_std, losses


def predict_sgpr(model, likelihood, test_x, x_mean, x_std, y_mean, y_std):
    """Predict with trained SGPR model."""
    test_x_norm = (test_x - x_mean) / x_std
    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        pred = likelihood(model(test_x_norm))
    pred_mean = (pred.mean * y_std + y_mean).numpy()
    pred_std = (pred.stddev * y_std).numpy()
    return pred_mean, pred_std


# ============================================================
# 3. Main training pipeline
# ============================================================

def main():
    bag_dir = "data_set/2026_05_05_free_flight/01_leveled_marker"

    print("=" * 60)
    print("System Dynamics Residual Learning with Sparse GPR")
    print("=" * 60)
    print(f"  S550: mass={MASS} kg, Ixx={IXX}, Iyy={IYY}, Izz={IZZ}")

    # --- Extract data ---
    print("\n[1] Extracting data from rosbag...")
    odom, ctrl, imu, rpm = extract_all_data(bag_dir)
    print(f"    odom:    {len(odom['ts'])} samples")
    print(f"    control: {len(ctrl['ts'])} samples")
    print(f"    imu:     {len(imu['ts'])} samples")
    print(f"    rpm:     {len(rpm['ts'])} samples")

    # --- Fz from actual RPM ---
    fz_rpm = CT * np.sum(rpm["vals"] ** 2, axis=1)
    print(f"    Fz from RPM: mean={fz_rpm.mean():.2f} N, mg={MASS*G:.2f} N")

    # --- Common time reference ---
    t0 = min(odom["ts"][0], ctrl["ts"][0], imu["ts"][0], rpm["ts"][0])
    odom_t = odom["ts"] - t0
    ctrl_t = ctrl["ts"] - t0
    rpm_t = rpm["ts"] - t0

    # --- Euler angles from odom quaternion ---
    roll, pitch, yaw = compute_euler_from_quat(
        odom["qx"], odom["qy"], odom["qz"], odom["qw"]
    )

    # --- Identify hovering segment ---
    z0 = odom["pz"][0]
    z_rel = odom["pz"] - z0
    hover_mask = z_rel > HOVER_Z_THRESHOLD
    hover_idx = np.where(hover_mask)[0]
    print(f"\n[2] Hovering segment (z_rel > {HOVER_Z_THRESHOLD}m):")
    print(f"    t=[{odom_t[hover_idx[0]]:.2f}, {odom_t[hover_idx[-1]]:.2f}] s")
    print(f"    {len(hover_idx)} odom samples")

    # --- Compute actual accelerations (world frame, from odom velocity) ---
    print("\n[3] Computing actual accelerations from odom velocity...")

    # odom twist is in body frame — transform to world frame
    vx_world = np.zeros(len(odom_t))
    vy_world = np.zeros(len(odom_t))
    vz_world = np.zeros(len(odom_t))

    quats = np.stack([odom["qx"], odom["qy"], odom["qz"], odom["qw"]], axis=1)
    rots = Rotation.from_quat(quats)
    for i in range(len(odom_t)):
        v_body = np.array([odom["vx"][i], odom["vy"][i], odom["vz"][i]])
        v_world = rots[i].apply(v_body)
        vx_world[i] = v_world[0]
        vy_world[i] = v_world[1]
        vz_world[i] = v_world[2]

    ax_actual, ay_actual, az_actual = compute_accel_from_velocity(
        odom_t, vx_world, vy_world, vz_world
    )

    # --- Interpolate to odom timestamps ---
    print("\n[4] Interpolating inputs to odom timestamps...")
    fz_interp = interp1d(rpm_t, fz_rpm, kind="linear",
                         fill_value="extrapolate", bounds_error=False)(odom_t)
    tx_interp = interp1d(ctrl_t, ctrl["tx"], kind="previous",
                         fill_value="extrapolate", bounds_error=False)(odom_t)
    ty_interp = interp1d(ctrl_t, ctrl["ty"], kind="previous",
                         fill_value="extrapolate", bounds_error=False)(odom_t)
    tz_interp = interp1d(ctrl_t, ctrl["tz"], kind="previous",
                         fill_value="extrapolate", bounds_error=False)(odom_t)

    # --- Rigid body prediction (Fz from actual RPM) ---
    print("\n[5] Computing rigid-body predicted accelerations (Fz from actual RPM)...")
    ax_pred, ay_pred, az_pred = predict_rigid_body_accel(fz_interp, roll, pitch)

    # --- Compute residuals (hovering only) ---
    res_ax = ax_actual[hover_idx] - ax_pred[hover_idx]
    res_ay = ay_actual[hover_idx] - ay_pred[hover_idx]
    res_az = az_actual[hover_idx] - az_pred[hover_idx]

    print(f"\n[6] Residual stats (hovering, {len(hover_idx)} samples):")
    print(f"    ax: mean={res_ax.mean():.4f}, std={res_ax.std():.4f} m/s^2")
    print(f"    ay: mean={res_ay.mean():.4f}, std={res_ay.std():.4f} m/s^2")
    print(f"    az: mean={res_az.mean():.4f}, std={res_az.std():.4f} m/s^2")

    # --- Prepare features ---
    # [Fz, tx, ty, tz, roll, pitch, vx_world, vy_world, vz_world]
    features = np.stack([
        fz_interp[hover_idx],
        tx_interp[hover_idx],
        ty_interp[hover_idx],
        tz_interp[hover_idx],
        roll[hover_idx],
        pitch[hover_idx],
        vx_world[hover_idx],
        vy_world[hover_idx],
        vz_world[hover_idx],
    ], axis=1)

    print(f"\n[7] Feature matrix: {features.shape}")

    # Remove NaN
    valid = np.isfinite(features).all(axis=1)
    valid &= np.isfinite(res_ax) & np.isfinite(res_ay) & np.isfinite(res_az)
    features = features[valid]
    res_ax = res_ax[valid]
    res_ay = res_ay[valid]
    res_az = res_az[valid]
    print(f"    Valid samples: {len(features)}")

    # Subsample if needed
    max_train = 3000
    if len(features) > max_train:
        idx = np.random.choice(len(features), max_train, replace=False)
        features_sub = features[idx]
        res_ax_sub = res_ax[idx]
        res_ay_sub = res_ay[idx]
        res_az_sub = res_az[idx]
        print(f"    Subsampled to: {max_train}")
    else:
        features_sub = features
        res_ax_sub = res_ax
        res_ay_sub = res_ay
        res_az_sub = res_az

    train_x = torch.tensor(features_sub, dtype=torch.float32)

    # --- Train one GP per axis ---
    save_dir = Path("data_set/trained_models")
    save_dir.mkdir(parents=True, exist_ok=True)

    models = {}
    axis_names = ["ax", "ay", "az"]
    residuals_sub = [res_ax_sub, res_ay_sub, res_az_sub]
    residuals_all = [res_ax, res_ay, res_az]
    all_losses = {}

    for ax_name, res_sub in zip(axis_names, residuals_sub):
        print(f"\n[8] Training Sparse GPR for {ax_name} residual...")
        train_y = torch.tensor(res_sub, dtype=torch.float32)

        model, lik, xm, xs, ym, ys, losses = train_sgpr(
            train_x, train_y, n_inducing=150, n_epochs=500, lr=0.01
        )
        models[ax_name] = (model, lik, xm, xs, ym, ys)
        all_losses[ax_name] = losses

        # Evaluate on training set
        pred_mean, pred_std = predict_sgpr(model, lik, train_x, xm, xs, ym, ys)
        err = res_sub - pred_mean
        print(f"      {ax_name} prediction error: mean={err.mean():.4f}, std={err.std():.4f}")
        print(f"      GP noise: {lik.noise.item():.6f}")

    # --- Save models ---
    save_dict = {"mass": MASS, "g": G}
    for ax_name in axis_names:
        model, lik, xm, xs, ym, ys = models[ax_name]
        save_dict[f"{ax_name}_model_state"] = model.state_dict()
        save_dict[f"{ax_name}_likelihood_state"] = lik.state_dict()
        save_dict[f"{ax_name}_x_mean"] = xm
        save_dict[f"{ax_name}_x_std"] = xs
        save_dict[f"{ax_name}_y_mean"] = ym
        save_dict[f"{ax_name}_y_std"] = ys

    save_path = save_dir / "system_sparse_gpr.pth"
    torch.save(save_dict, save_path)
    print(f"\n    Models saved to: {save_path}")

    # --- Plots ---
    print("\n[9] Generating plots...")

    # Use full hovering data for prediction
    test_x = torch.tensor(features, dtype=torch.float32)
    hover_t = odom_t[hover_idx][valid]

    fig, axes = plt.subplots(3, 2, figsize=(16, 12))

    accel_actual = [ax_actual[hover_idx][valid],
                    ay_actual[hover_idx][valid],
                    az_actual[hover_idx][valid]]
    accel_pred_rb = [ax_pred[hover_idx][valid],
                     ay_pred[hover_idx][valid],
                     az_pred[hover_idx][valid]]
    labels = ["ax (m/s²)", "ay (m/s²)", "az (m/s²)"]

    for i, (ax_name, label) in enumerate(zip(axis_names, labels)):
        model, lik, xm, xs, ym, ys = models[ax_name]
        gp_correction, gp_std = predict_sgpr(model, lik, test_x, xm, xs, ym, ys)
        corrected = accel_pred_rb[i] + gp_correction

        # Left: time series
        ax = axes[i, 0]
        ax.plot(hover_t, accel_actual[i], "k.", markersize=1, alpha=0.3, label="Actual")
        ax.plot(hover_t, accel_pred_rb[i], "b-", alpha=0.4, linewidth=0.8, label="Rigid Body")
        ax.plot(hover_t, corrected, "r-", alpha=0.5, linewidth=0.8, label="RB + GP")
        ax.set_ylabel(label)
        ax.set_xlabel("Time (s)")
        ax.legend(fontsize=7)
        ax.set_title(f"{ax_name} — Time Series")

        # Right: residual before/after
        ax2 = axes[i, 1]
        res_before = residuals_all[i]
        res_after = accel_actual[i] - corrected
        ax2.hist(res_before, bins=80, alpha=0.5, density=True, label=f"Before GP (std={res_before.std():.3f})")
        ax2.hist(res_after, bins=80, alpha=0.5, density=True, label=f"After GP (std={res_after.std():.3f})")
        ax2.set_xlabel(f"Residual {label}")
        ax2.set_ylabel("Density")
        ax2.legend(fontsize=7)
        ax2.set_title(f"{ax_name} — Residual Distribution")

    plt.suptitle("System Dynamics Residual Learning (Hover)", fontsize=14)
    plt.tight_layout()
    plt.savefig(str(save_dir / "system_sparse_gpr_results.png"), dpi=150)
    print(f"    Results saved to: {save_dir / 'system_sparse_gpr_results.png'}")

    # Loss curves
    fig2, axes2 = plt.subplots(1, 3, figsize=(15, 4))
    for i, ax_name in enumerate(axis_names):
        axes2[i].plot(all_losses[ax_name])
        axes2[i].set_xlabel("Epoch")
        axes2[i].set_ylabel("Neg. ELBO")
        axes2[i].set_title(f"{ax_name} Training Loss")
    plt.tight_layout()
    plt.savefig(str(save_dir / "system_sparse_gpr_loss.png"), dpi=150)
    print(f"    Loss curves saved to: {save_dir / 'system_sparse_gpr_loss.png'}")

    # Predicted vs actual scatter
    fig3, axes3 = plt.subplots(1, 3, figsize=(15, 4))
    for i, (ax_name, label) in enumerate(zip(axis_names, labels)):
        model, lik, xm, xs, ym, ys = models[ax_name]
        gp_correction, _ = predict_sgpr(model, lik, test_x, xm, xs, ym, ys)
        corrected = accel_pred_rb[i] + gp_correction

        axes3[i].plot(accel_actual[i], corrected, ".", markersize=1, alpha=0.3)
        lims = [min(accel_actual[i].min(), corrected.min()),
                max(accel_actual[i].max(), corrected.max())]
        axes3[i].plot(lims, lims, "r--", linewidth=0.8)
        axes3[i].set_xlabel(f"Actual {label}")
        axes3[i].set_ylabel(f"Predicted {label}")
        axes3[i].set_title(f"{ax_name} — Actual vs Corrected")
        axes3[i].set_aspect("equal")
    plt.tight_layout()
    plt.savefig(str(save_dir / "system_sparse_gpr_scatter.png"), dpi=150)
    print(f"    Scatter plots saved to: {save_dir / 'system_sparse_gpr_scatter.png'}")

    print("\n" + "=" * 60)
    print("Done!")
    print("=" * 60)


if __name__ == "__main__":
    main()
