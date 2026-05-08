"""
Motor Model Residual Learning with Sparse GPR
==============================================
Learns the residual between pure 2nd-order motor model and actual RPM data.

Input: cmd_rpm (from cmd_raw * 9800/8191)
Output: actual_rpm
Residual: actual_rpm - model_prediction(cmd_rpm)

Uses GPyTorch Variational SGPR with inducing points.
"""

import sqlite3
import struct
import numpy as np
import torch
import gpytorch
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d

# ============================================================
# 1. Data extraction from rosbag
# ============================================================

MAX_RPM = 9800.0
MAX_BIT = 8191.0


def extract_bag_data(bag_dir: str):
    """Extract cmd_raw and actual_rpm from a rosbag2 sqlite3 file."""
    bag_path = Path(bag_dir)
    db_files = list(bag_path.glob("*.db3"))
    if not db_files:
        raise FileNotFoundError(f"No .db3 files found in {bag_path}")

    conn = sqlite3.connect(str(db_files[0]))

    topics = conn.execute("SELECT id, name, type FROM topics").fetchall()
    topic_map = {t[1]: t[0] for t in topics}

    cmd_raw_id = topic_map["/uav/cmd_raw"]
    actual_rpm_id = topic_map["/uav/actual_rpm"]

    # Parse HexaCmdRaw: Header + int16[6] cmd_raw
    # CDR layout: [4B CDR hdr][4B sec][4B nsec][4B frame_len][frame_id bytes][align2][6×int16]
    cmd_timestamps = []
    cmd_values = []
    rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (cmd_raw_id,),
    ).fetchall()

    for ts, data in rows:
        offset = 4
        sec, nsec = struct.unpack_from("<II", data, offset)
        offset += 8
        frame_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4 + frame_len
        if offset % 2:
            offset += 1
        vals = struct.unpack_from("<6h", data, offset)
        t = sec + nsec * 1e-9
        cmd_timestamps.append(t)
        cmd_values.append(list(vals))

    # Parse HexaActualRpm: Header + int32[6] rpm + int32[6] acceleration
    # CDR layout: [4B CDR hdr][4B sec][4B nsec][4B frame_len][frame_id bytes][align4][6×int32][6×int32]
    rpm_timestamps = []
    rpm_values = []
    rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (actual_rpm_id,),
    ).fetchall()

    for ts, data in rows:
        offset = 4
        sec, nsec = struct.unpack_from("<II", data, offset)
        offset += 8
        frame_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4 + frame_len
        while offset % 4:
            offset += 1
        vals = struct.unpack_from("<6i", data, offset)
        t = sec + nsec * 1e-9
        rpm_timestamps.append(t)
        rpm_values.append(list(vals))

    conn.close()

    cmd_timestamps = np.array(cmd_timestamps)
    cmd_values = np.array(cmd_values, dtype=np.float64)
    rpm_timestamps = np.array(rpm_timestamps)
    rpm_values = np.array(rpm_values, dtype=np.float64)

    cmd_rpm = cmd_values * (MAX_RPM / MAX_BIT)

    return cmd_timestamps, cmd_rpm, rpm_timestamps, rpm_values


# ============================================================
# 2. Pure 2nd-order motor model simulation
# ============================================================

class PureSecondOrderModel:
    """Pure 2nd-order transfer function: omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)"""

    def __init__(self, zeta=0.925, omega_n=22.42):
        self.zeta = zeta
        self.omega_n = omega_n

    def simulate(self, timestamps, cmd_rpm_per_motor, rpm_init=0.0):
        N = len(timestamps)
        rpm = np.zeros(N)
        rpm[0] = rpm_init
        rpm_dot = np.zeros(N)

        wn = self.omega_n
        wn2 = wn * wn
        z = self.zeta

        for i in range(1, N):
            dt = timestamps[i] - timestamps[i - 1]
            if dt <= 0 or dt > 0.1:
                continue

            def dynamics(w, w_dot, cmd):
                w_ddot = wn2 * (cmd - w) - 2.0 * z * wn * w_dot
                return w_dot, w_ddot

            k1_w, k1_wd = dynamics(rpm[i-1], rpm_dot[i-1], cmd_rpm_per_motor[i-1])
            k2_w, k2_wd = dynamics(rpm[i-1] + 0.5*dt*k1_w, rpm_dot[i-1] + 0.5*dt*k1_wd, cmd_rpm_per_motor[i-1])
            k3_w, k3_wd = dynamics(rpm[i-1] + 0.5*dt*k2_w, rpm_dot[i-1] + 0.5*dt*k2_wd, cmd_rpm_per_motor[i-1])
            k4_w, k4_wd = dynamics(rpm[i-1] + dt*k3_w, rpm_dot[i-1] + dt*k3_wd, cmd_rpm_per_motor[i-1])

            rpm[i] = rpm[i-1] + dt/6.0 * (k1_w + 2*k2_w + 2*k3_w + k4_w)
            rpm_dot[i] = rpm_dot[i-1] + dt/6.0 * (k1_wd + 2*k2_wd + 2*k3_wd + k4_wd)

            rpm[i] = np.clip(rpm[i], -15000, 15000)
            rpm_dot[i] = np.clip(rpm_dot[i], -500000, 500000)

        return rpm


# ============================================================
# 3. Sparse GPR Model (GPyTorch Variational SGPR)
# ============================================================

class SparseGPModel(gpytorch.models.ApproximateGP):
    """Variational Sparse GP (SGPR) — numerically stable."""

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
            gpytorch.kernels.RBFKernel()
        )

    def forward(self, x):
        mean = self.mean_module(x)
        covar = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean, covar)


# ============================================================
# 4. Main training pipeline
# ============================================================

def main():
    bag_dir = "data_set/2026_05_05_free_flight/01_leveled_marker"

    print("=" * 60)
    print("Motor Model Residual Learning with Sparse GPR")
    print("=" * 60)

    # --- Extract data ---
    print("\n[1] Extracting data from rosbag...")
    cmd_ts, cmd_rpm, rpm_ts, actual_rpm = extract_bag_data(bag_dir)
    print(f"    cmd_raw:    {len(cmd_ts)} samples, t=[{cmd_ts[0]:.1f}, {cmd_ts[-1]:.1f}]")
    print(f"    actual_rpm: {len(rpm_ts)} samples, t=[{rpm_ts[0]:.1f}, {rpm_ts[-1]:.1f}]")

    # --- Align timestamps ---
    print("\n[2] Aligning cmd_rpm to actual_rpm timestamps...")
    t0 = min(cmd_ts[0], rpm_ts[0])
    cmd_ts_rel = cmd_ts - t0
    rpm_ts_rel = rpm_ts - t0

    cmd_rpm_interp = np.zeros((len(rpm_ts), 6))
    for motor_i in range(6):
        first_val = cmd_rpm[0, motor_i]
        f = interp1d(cmd_ts_rel, cmd_rpm[:, motor_i], kind="previous",
                     fill_value=(first_val, cmd_rpm[-1, motor_i]),
                     bounds_error=False)
        cmd_rpm_interp[:, motor_i] = f(rpm_ts_rel)

    # --- Simulate pure 2nd-order model ---
    print("\n[3] Simulating pure 2nd-order motor model (zeta=0.925, omega_n=22.42)...")
    model_sim = PureSecondOrderModel(zeta=0.925, omega_n=22.42)

    model_rpm = np.zeros_like(actual_rpm)
    for motor_i in range(6):
        sim_result = model_sim.simulate(
            rpm_ts_rel, cmd_rpm_interp[:, motor_i],
            rpm_init=actual_rpm[0, motor_i])
        model_rpm[:, motor_i] = sim_result

    # --- Compute residual ---
    residual = actual_rpm - model_rpm

    print(f"    Residual stats (all motors):")
    print(f"    mean={residual.mean():.2f}, std={residual.std():.2f}, "
          f"min={residual.min():.2f}, max={residual.max():.2f}")

    # --- Prepare training data (pool all 6 motors) ---
    print("\n[4] Preparing training data...")

    train_features = []
    train_targets = []
    for motor_i in range(6):
        cmd_i = cmd_rpm_interp[:, motor_i]
        model_i = model_rpm[:, motor_i]
        res_i = residual[:, motor_i]

        features = np.stack([cmd_i, model_i], axis=1)
        train_features.append(features)
        train_targets.append(res_i)

    train_features = np.concatenate(train_features, axis=0)
    train_targets = np.concatenate(train_targets, axis=0)

    print(f"    Training samples: {len(train_targets)}")
    print(f"    Feature shape: {train_features.shape}")

    valid_mask = np.isfinite(train_features).all(axis=1) & np.isfinite(train_targets)
    train_features = train_features[valid_mask]
    train_targets = train_targets[valid_mask]
    print(f"    Valid samples after NaN removal: {len(train_targets)}")

    max_train = 5000
    if len(train_targets) > max_train:
        idx = np.random.choice(len(train_targets), max_train, replace=False)
        train_features = train_features[idx]
        train_targets = train_targets[idx]
        print(f"    Subsampled to: {len(train_targets)}")

    train_x = torch.tensor(train_features, dtype=torch.float32)
    train_y = torch.tensor(train_targets, dtype=torch.float32)

    # Normalize features
    x_mean = train_x.mean(dim=0)
    x_std = train_x.std(dim=0)
    x_std[x_std < 1e-6] = 1.0
    train_x_norm = (train_x - x_mean) / x_std

    # Normalize targets
    y_mean = train_y.mean()
    y_std = train_y.std()
    if y_std < 1e-6:
        y_std = torch.tensor(1.0)
    train_y_norm = (train_y - y_mean) / y_std

    # --- Train Variational Sparse GPR ---
    print("\n[5] Training Variational Sparse GPR...")
    n_inducing = min(200, len(train_targets) // 5)
    print(f"    Inducing points: {n_inducing}")

    inducing_idx = torch.randperm(train_x_norm.size(0))[:n_inducing]
    inducing_points = train_x_norm[inducing_idx].clone()

    likelihood = gpytorch.likelihoods.GaussianLikelihood()
    gp_model = SparseGPModel(inducing_points)

    gp_model.train()
    likelihood.train()

    optimizer = torch.optim.Adam(
        list(gp_model.parameters()) + list(likelihood.parameters()), lr=0.01
    )
    mll = gpytorch.mlls.VariationalELBO(likelihood, gp_model, num_data=train_x_norm.size(0))

    n_epochs = 500
    losses = []
    for epoch in range(n_epochs):
        optimizer.zero_grad()
        output = gp_model(train_x_norm)
        loss = -mll(output, train_y_norm)
        loss.backward()
        optimizer.step()
        losses.append(loss.item())
        if (epoch + 1) % 100 == 0:
            print(f"    Epoch {epoch+1}/{n_epochs}, Loss: {loss.item():.4f}")

    # --- Evaluate ---
    print("\n[6] Evaluating...")
    gp_model.eval()
    likelihood.eval()

    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        pred = likelihood(gp_model(train_x_norm))
        pred_mean_norm = pred.mean
        pred_std_norm = pred.stddev

    # Denormalize predictions
    pred_mean = (pred_mean_norm * y_std + y_mean).numpy()
    pred_std = (pred_std_norm * y_std).numpy()

    train_residual_error = train_targets - pred_mean
    print(f"    Prediction error: mean={train_residual_error.mean():.4f}, "
          f"std={train_residual_error.std():.4f}")
    print(f"    GP noise: {likelihood.noise.item():.6f}")
    print(f"    GP lengthscale: {gp_model.covar_module.base_kernel.lengthscale.detach().numpy()}")
    print(f"    GP outputscale: {gp_model.covar_module.outputscale.item():.4f}")

    # --- Save model ---
    save_dir = Path("data_set/trained_models")
    save_dir.mkdir(parents=True, exist_ok=True)

    save_path = save_dir / "motor_sparse_gpr.pth"
    torch.save({
        "model_state": gp_model.state_dict(),
        "likelihood_state": likelihood.state_dict(),
        "x_mean": x_mean,
        "x_std": x_std,
        "y_mean": y_mean,
        "y_std": y_std,
        "n_inducing": n_inducing,
        "zeta": 0.925,
        "omega_n": 22.42,
    }, save_path)
    print(f"\n    Model saved to: {save_path}")

    # --- Plot results ---
    print("\n[7] Generating plots...")
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    for motor_i in range(6):
        ax = axes[motor_i // 2, motor_i % 2]

        actual_m = actual_rpm[:, motor_i]
        model_m = model_rpm[:, motor_i]

        # GP correction for this motor
        feat = np.stack([cmd_rpm_interp[:, motor_i], model_m], axis=1)
        feat_t = torch.tensor(feat, dtype=torch.float32)
        feat_norm = (feat_t - x_mean) / x_std

        with torch.no_grad(), gpytorch.settings.fast_pred_var():
            gp_pred = likelihood(gp_model(feat_norm))
            correction_norm = gp_pred.mean
        correction = (correction_norm * y_std + y_mean).numpy()

        corrected = model_m + correction

        ax.plot(rpm_ts_rel, actual_m, "k.", markersize=1, alpha=0.3, label="Actual")
        ax.plot(rpm_ts_rel, model_m, "b-", alpha=0.5, linewidth=0.8, label="Model")
        ax.plot(rpm_ts_rel, corrected, "r-", alpha=0.5, linewidth=0.8, label="Model+GP")
        ax.set_title(f"Motor {motor_i+1}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("RPM")
        ax.legend(fontsize=7)

    plt.suptitle("Motor Model Residual Learning (Sparse GPR)", fontsize=14)
    plt.tight_layout()
    plt.savefig(str(save_dir / "motor_sparse_gpr_results.png"), dpi=150)
    print(f"    Plot saved to: {save_dir / 'motor_sparse_gpr_results.png'}")

    # Loss curve
    fig2, ax2 = plt.subplots(1, 1, figsize=(8, 4))
    ax2.plot(losses)
    ax2.set_xlabel("Epoch")
    ax2.set_ylabel("Neg. ELBO Loss")
    ax2.set_title("Training Loss")
    plt.tight_layout()
    plt.savefig(str(save_dir / "motor_sparse_gpr_loss.png"), dpi=150)
    print(f"    Loss plot saved to: {save_dir / 'motor_sparse_gpr_loss.png'}")

    # Residual histogram
    fig3, ax3 = plt.subplots(1, 1, figsize=(8, 4))
    ax3.hist(train_targets, bins=100, alpha=0.5, label="Before GP", density=True)
    ax3.hist(train_residual_error, bins=100, alpha=0.5, label="After GP", density=True)
    ax3.set_xlabel("Residual (RPM)")
    ax3.set_ylabel("Density")
    ax3.set_title("Residual Distribution")
    ax3.legend()
    plt.tight_layout()
    plt.savefig(str(save_dir / "motor_residual_histogram.png"), dpi=150)
    print(f"    Histogram saved to: {save_dir / 'motor_residual_histogram.png'}")

    print("\n" + "=" * 60)
    print("Done!")
    print("=" * 60)


if __name__ == "__main__":
    main()
