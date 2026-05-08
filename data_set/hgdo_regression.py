"""
HGDO Disturbance Regression Analysis
=====================================
Estimates CG offset (p_off) and viscous damping (b) from
HGDO wrench data during hovering flight.

Model: tau_hgdo = -p_off x F_col + b * omega

Uses Ct = 1.255e-7 N/(rpm)^2 to compute F_col from actual RPM.
"""

import numpy as np
import struct
import sqlite3
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from sklearn.metrics import r2_score

# ============================================================
# Parameters
# ============================================================

MASS = 3.188      # kg
G = 9.81          # m/s^2
CT = 1.255e-7     # N/(rpm)^2
HOVER_Z_THRESHOLD = 0.05  # m


def extract_data(bag_dir: str):
    bag_path = Path(bag_dir)
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    odom = {"ts": [], "pz": [], "wx": [], "wy": [], "wz": []}
    hgdo = {"ts": [], "tx": [], "ty": [], "tz": []}

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == "/mavros/local_position/odom":
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                odom["ts"].append(t)
                odom["pz"].append(msg.pose.pose.position.z)
                odom["wx"].append(msg.twist.twist.angular.x)
                odom["wy"].append(msg.twist.twist.angular.y)
                odom["wz"].append(msg.twist.twist.angular.z)
            elif connection.topic == "/hgdo/wrench":
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                hgdo["ts"].append(t)
                hgdo["tx"].append(msg.wrench.torque.x)
                hgdo["ty"].append(msg.wrench.torque.y)
                hgdo["tz"].append(msg.wrench.torque.z)

    # Parse actual_rpm from sqlite
    db_files = list(bag_path.glob("*.db3"))
    conn = sqlite3.connect(str(db_files[0]))
    topics = conn.execute("SELECT id, name, type FROM topics").fetchall()
    topic_map = {t[1]: t[0] for t in topics}
    rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_map["/uav/actual_rpm"],),
    ).fetchall()

    rpm = {"ts": [], "vals": []}
    for ts, data in rows:
        offset = 4
        sec, nsec = struct.unpack_from("<II", data, offset)
        offset += 8
        frame_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4 + frame_len
        while offset % 4:
            offset += 1
        vals = struct.unpack_from("<6i", data, offset)
        rpm["ts"].append(sec + nsec * 1e-9)
        rpm["vals"].append(list(vals))
    conn.close()

    for key in odom:
        odom[key] = np.array(odom[key])
    for key in hgdo:
        hgdo[key] = np.array(hgdo[key])
    rpm["ts"] = np.array(rpm["ts"])
    rpm["vals"] = np.array(rpm["vals"], dtype=np.float64)

    return odom, hgdo, rpm


def main():
    bag_dir = "data_set/2026_05_05_free_flight/01_leveled_marker"

    print("=" * 60)
    print("HGDO Disturbance Regression")
    print("=" * 60)

    # --- Extract ---
    odom, hgdo, rpm = extract_data(bag_dir)

    # Fz from RPM
    fz_rpm = CT * np.sum(rpm["vals"] ** 2, axis=1)

    t0 = min(odom["ts"][0], hgdo["ts"][0], rpm["ts"][0])
    odom_t = odom["ts"] - t0
    hgdo_t = hgdo["ts"] - t0
    rpm_t = rpm["ts"] - t0

    print(f"  Fz from RPM: mean={fz_rpm.mean():.2f} N, mg={MASS*G:.2f} N")

    # --- Hover segment ---
    z_rel = odom["pz"] - odom["pz"][0]
    hover_idx = np.where(z_rel > HOVER_Z_THRESHOLD)[0]
    t_start = odom_t[hover_idx[0]]
    t_end = odom_t[hover_idx[-1]]
    h = np.where((hgdo_t >= t_start) & (hgdo_t <= t_end))[0]
    t_h = hgdo_t[h]
    print(f"  Hover: {len(h)} HGDO samples, t=[{t_start:.1f}, {t_end:.1f}]s")

    # --- Interpolate to HGDO timestamps ---
    wx = interp1d(odom_t, odom["wx"], kind="linear",
                  fill_value="extrapolate", bounds_error=False)(t_h)
    wy = interp1d(odom_t, odom["wy"], kind="linear",
                  fill_value="extrapolate", bounds_error=False)(t_h)
    wz = interp1d(odom_t, odom["wz"], kind="linear",
                  fill_value="extrapolate", bounds_error=False)(t_h)
    fz = interp1d(rpm_t, fz_rpm, kind="linear",
                  fill_value="extrapolate", bounds_error=False)(t_h)

    tx = hgdo["tx"][h]
    ty = hgdo["ty"][h]
    tz = hgdo["tz"][h]

    # --- Regression ---
    # Roll:  tau_x = (-y_off) * Fz + b_x * wx
    # Pitch: tau_y = ( x_off) * Fz + b_y * wy
    # Yaw:   tau_z = b_z * wz + c_z

    A_x = np.column_stack([fz, wx])
    coeffs_x, _, _, _ = np.linalg.lstsq(A_x, tx, rcond=None)
    y_off = -coeffs_x[0]
    b_x = coeffs_x[1]

    A_y = np.column_stack([fz, wy])
    coeffs_y, _, _, _ = np.linalg.lstsq(A_y, ty, rcond=None)
    x_off = coeffs_y[0]
    b_y = coeffs_y[1]

    A_z = np.column_stack([wz, np.ones(len(wz))])
    coeffs_z, _, _, _ = np.linalg.lstsq(A_z, tz, rcond=None)
    b_z = coeffs_z[0]
    c_z = coeffs_z[1]

    tx_pred = A_x @ coeffs_x
    ty_pred = A_y @ coeffs_y
    tz_pred = A_z @ coeffs_z

    res_tx = tx - tx_pred
    res_ty = ty - ty_pred
    res_tz = tz - tz_pred

    r2_x = r2_score(tx, tx_pred)
    r2_y = r2_score(ty, ty_pred)
    r2_z = r2_score(tz, tz_pred)

    print(f"\n{'='*60}")
    print(f"Results (Ct = {CT})")
    print(f"{'='*60}")
    print(f"  Roll:  y_off = {y_off*1000:.3f} mm,  b_x = {b_x:.4f} Nm·s/rad,  R² = {r2_x:.4f}")
    print(f"  Pitch: x_off = {x_off*1000:.3f} mm,  b_y = {b_y:.4f} Nm·s/rad,  R² = {r2_y:.4f}")
    print(f"  Yaw:   b_z = {b_z:.4f} Nm·s/rad,  c_z = {c_z:.6f} Nm,  R² = {r2_z:.4f}")
    print(f"\n  Residual std:  tx={res_tx.std():.4f},  ty={res_ty.std():.4f},  tz={res_tz.std():.4f} Nm")
    print(f"  Fz hover mean: {fz.mean():.2f} N")

    # --- Plot ---
    fig, axes = plt.subplots(3, 3, figsize=(18, 10))

    for i, (w_val, tau_val, tau_pred_val, bval, fz_coeff, r2, name) in enumerate([
        (wx, tx, tx_pred, b_x, coeffs_x[0], r2_x, r"Roll: $\tau_x$ vs $\omega_x$"),
        (wy, ty, ty_pred, b_y, coeffs_y[0], r2_y, r"Pitch: $\tau_y$ vs $\omega_y$"),
        (wz, tz, tz_pred, b_z, None, r2_z, r"Yaw: $\tau_z$ vs $\omega_z$"),
    ]):
        ax = axes[i, 0]
        ax.plot(w_val, tau_val, ".", ms=1, alpha=0.3, color="steelblue")
        w_range = np.linspace(w_val.min(), w_val.max(), 100)
        if i < 2:
            fit_line = fz_coeff * fz.mean() + bval * w_range
        else:
            fit_line = bval * w_range + c_z
        ax.plot(w_range, fit_line, "r-", lw=2, label=f"b={bval:.4f}\nR²={r2:.4f}")
        ax.set_xlabel(r"$\omega$ (rad/s)")
        ax.set_ylabel("HGDO torque (Nm)")
        ax.set_title(name)
        ax.legend(fontsize=8)

    for i, (tau_val, pred_val, name) in enumerate([
        (tx, tx_pred, r"$\tau_x$: data vs fit"),
        (ty, ty_pred, r"$\tau_y$: data vs fit"),
        (tz, tz_pred, r"$\tau_z$: data vs fit"),
    ]):
        ax = axes[i, 1]
        ax.plot(t_h, tau_val, "b-", lw=0.5, alpha=0.5, label="HGDO data")
        ax.plot(t_h, pred_val, "r-", lw=1.0, alpha=0.8, label="Fit")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Torque (Nm)")
        ax.set_title(name)
        ax.legend(fontsize=8)

    for i, (res, name) in enumerate([
        (res_tx, r"$\tau_x$ residual"),
        (res_ty, r"$\tau_y$ residual"),
        (res_tz, r"$\tau_z$ residual"),
    ]):
        ax = axes[i, 2]
        ax.hist(res, bins=80, alpha=0.7, density=True, color="steelblue")
        ax.set_xlabel("Residual (Nm)")
        ax.set_ylabel("Density")
        ax.set_title(f"{name} (std={res.std():.4f})")

    plt.suptitle(
        r"HGDO Regression: $\tau = -\mathbf{p}_{off} \times \mathbf{F}_{col}"
        r" + \mathbf{b} \cdot \omega$"
        f"\nx_off={x_off*1000:.2f}mm, y_off={y_off*1000:.2f}mm, "
        f"b_x={b_x:.4f}, b_y={b_y:.4f}, b_z={b_z:.4f} Nm·s/rad",
        fontsize=12,
    )
    plt.tight_layout()

    save_dir = Path("data_set/trained_models")
    save_dir.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(save_dir / "hgdo_regression_ct1255.png"), dpi=150)
    print(f"\n  Plot saved to: {save_dir / 'hgdo_regression_ct1255.png'}")


if __name__ == "__main__":
    main()
