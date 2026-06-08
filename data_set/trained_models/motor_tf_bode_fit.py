#!/usr/bin/env python3
"""
Motor Transfer Function Bode Fitting with -3dB Cutoff-First Methodology.

Model: H(s) = K * ωn² / (s² + 2ζωn s + ωn²) · exp(-τd s)

Data: 2026-05-05 01_leveled_marker (real flight, highest coherence)

Steps:
  1. Parse cmd_raw (PWM) and actual_rpm (RPM) from rosbag
  2. Resample to uniform dt, compute CSD-based H1 estimator and coherence
  3. Find empirical -3dB bandwidth from |H_norm(f)|
  4. Constrain ωn via: u = 1-2ζ²+√(4ζ⁴-4ζ²+2), ωn = ω_{-3dB}/√u
  5. Fit (K, ζ, τd) via coherence-weighted least squares
  6. Generate Bode plot at dpi=600
"""

import sqlite3
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal as sig
from scipy.ndimage import uniform_filter1d
from scipy.optimize import differential_evolution

DB = '/home/user/drone_simulator_pkgs/data_set/2026_05_05_free_flight/01_leveled_marker/01_leveled_marker_0.db3'
OUTPATH = '/home/user/drone_simulator_pkgs/data_set/trained_models/motor_tf_bode_comparison.png'

# Paper reference values (ARX time-domain method)
Z_PAPER, WN_PAPER, TD_PAPER = 0.7814, 21.74, 0.01567

def parse_topic(dbpath, topic, dtype_fmt, offset, ncols):
    db = sqlite3.connect(dbpath)
    cur = db.cursor()
    cur.execute(
        'SELECT m.data, m.timestamp FROM messages m '
        'JOIN topics t ON m.topic_id=t.id '
        'WHERE t.name=? ORDER BY m.timestamp', (topic,))
    rows = cur.fetchall()
    db.close()
    n = len(rows)
    ts = np.zeros(n)
    vals = np.zeros((n, ncols))
    for i, (data, stamp) in enumerate(rows):
        ts[i] = stamp / 1e9
        vals[i] = struct.unpack_from(dtype_fmt, data, offset)
    return ts, vals

def model_H(f, K, zeta, omega_n, tau_d):
    s = 2j * np.pi * f
    return K * omega_n**2 / (s**2 + 2*zeta*omega_n*s + omega_n**2) * np.exp(-tau_d * s)

def omega_n_from_3dB(zeta, omega_3dB):
    u = 1 - 2*zeta**2 + np.sqrt(4*zeta**4 - 4*zeta**2 + 2)
    return omega_3dB / np.sqrt(u) if u > 0 else omega_3dB * 10

# ── Parse ──
print("Parsing 05-05 01_leveled_marker...")
t_cmd, cmd = parse_topic(DB, '/uav/cmd_raw', '<6H', 18, 6)
t_rpm, rpm = parse_topic(DB, '/uav/actual_rpm', '<6I', 20, 6)

cmd_avg = np.mean(cmd, axis=1).astype(float)
rpm_avg = np.mean(rpm, axis=1).astype(float)

# Hover window
hover = rpm_avg > 2000
i0, i1 = np.where(hover)[0][0], np.where(hover)[0][-1]
t_start = t_rpm[i0] + 1.0
t_end = t_rpm[i1]

cmd_mask = (t_cmd >= t_start) & (t_cmd <= t_end)
rpm_mask = (t_rpm >= t_start) & (t_rpm <= t_end)
dt = max(np.median(np.diff(t_cmd[cmd_mask])), np.median(np.diff(t_rpm[rpm_mask])))
fs = 1.0 / dt

t_u = np.arange(t_start, t_end, dt)
c_u = np.interp(t_u, t_cmd[cmd_mask], cmd_avg[cmd_mask])
r_u = np.interp(t_u, t_rpm[rpm_mask], rpm_avg[rpm_mask])
c_ac = c_u - np.mean(c_u)
r_ac = r_u - np.mean(r_u)
N = len(t_u)
K_static = np.mean(r_u) / np.mean(c_u)
print(f"  N={N}, fs={fs:.1f} Hz, K_static={K_static:.4f}")

# ── CSD ──
nperseg = 256
freqs, Sxx = sig.welch(c_ac, fs=fs, nperseg=nperseg, noverlap=nperseg//2)
_, Syy = sig.welch(r_ac, fs=fs, nperseg=nperseg, noverlap=nperseg//2)
_, Sxy = sig.csd(c_ac, r_ac, fs=fs, nperseg=nperseg, noverlap=nperseg//2)

H1 = Sxy / Sxx
coh = np.abs(Sxy)**2 / (Sxx * Syy + 1e-30)

H_mag = np.abs(H1)
H_dc = H_mag[1]
H_norm_dB = 20 * np.log10(H_mag / H_dc + 1e-30)

# ── Find -3dB ──
f_min, f_max = 0.3, 15.0
fmask = (freqs >= f_min) & (freqs <= f_max)
f_sel = freqs[fmask]
H_sel_dB = H_norm_dB[fmask]

below = np.where(H_sel_dB < -3.0)[0]
if len(below) > 0 and below[0] > 0:
    idx = below[0]
    f_a, f_b = f_sel[idx-1], f_sel[idx]
    m_a, m_b = H_sel_dB[idx-1], H_sel_dB[idx]
    f_3dB = f_a + (f_b - f_a) * (-3.0 - m_a) / (m_b - m_a)
else:
    f_3dB = f_sel[np.argmin(np.abs(H_sel_dB + 3.0))]

omega_3dB = 2 * np.pi * f_3dB
print(f"  f_-3dB = {f_3dB:.2f} Hz (ω_-3dB = {omega_3dB:.2f} rad/s)")

# ── -3dB constrained fit ──
H1_sel = H1[fmask]
coh_sel = coh[fmask]

def cost_3dB(params, freqs, H1_data, coh, omega_3dB):
    K, zeta, tau_d = params
    if zeta < 0.1 or zeta > 2.0 or tau_d < 0.0 or tau_d > 0.1 or K < 0:
        return 1e12
    omega_n = omega_n_from_3dB(zeta, omega_3dB)
    H_m = model_H(freqs, K, zeta, omega_n, tau_d)
    w = coh**2
    mag_err = np.sum(w * (20*np.log10(np.abs(H1_data)+1e-30) - 20*np.log10(np.abs(H_m)+1e-30))**2)
    ph_diff = np.angle(np.exp(1j*(np.angle(H1_data) - np.angle(H_m))))
    ph_err = np.sum(w * ph_diff**2)
    return mag_err + 0.5 * ph_err

result = differential_evolution(
    cost_3dB,
    bounds=[(K_static*0.5, K_static*2.0), (0.3, 1.5), (0.001, 0.060)],
    args=(f_sel, H1_sel, coh_sel, omega_3dB),
    seed=42, maxiter=2000, tol=1e-12, polish=True,
)
K_fit, z_fit, td_fit = result.x
wn_fit = omega_n_from_3dB(z_fit, omega_3dB)

# Also do free 4-param fit for comparison
def cost_free(params, freqs, H1_data, coh):
    K, zeta, omega_n, tau_d = params
    if zeta < 0.1 or zeta > 2.0 or tau_d < 0.0 or tau_d > 0.1 or K < 0 or omega_n < 5:
        return 1e12
    H_m = model_H(freqs, K, zeta, omega_n, tau_d)
    w = coh**2
    mag_err = np.sum(w * (20*np.log10(np.abs(H1_data)+1e-30) - 20*np.log10(np.abs(H_m)+1e-30))**2)
    ph_diff = np.angle(np.exp(1j*(np.angle(H1_data) - np.angle(H_m))))
    ph_err = np.sum(w * ph_diff**2)
    return mag_err + 0.5 * ph_err

result_free = differential_evolution(
    cost_free,
    bounds=[(K_static*0.5, K_static*2.0), (0.3, 1.5), (10, 60), (0.001, 0.060)],
    args=(f_sel, H1_sel, coh_sel),
    seed=42, maxiter=2000, tol=1e-12, polish=True,
)
K_free, z_free, wn_free, td_free = result_free.x

print(f"\n  -3dB constrained: K={K_fit:.4f}, ζ={z_fit:.4f}, ωn={wn_fit:.2f}, τd={td_fit*1000:.2f}ms (cost={result.fun:.2f})")
print(f"  Free 4-param:     K={K_free:.4f}, ζ={z_free:.4f}, ωn={wn_free:.2f}, τd={td_free*1000:.2f}ms (cost={result_free.fun:.2f})")
print(f"  Paper (ARX):      K≈{K_static:.4f}, ζ={Z_PAPER}, ωn={WN_PAPER}, τd={TD_PAPER*1000:.2f}ms")

# ── Bode Plot ──
f_model = np.linspace(0.1, f_max, 2000)
H_csd = model_H(f_model, K_fit, z_fit, wn_fit, td_fit)
H_free = model_H(f_model, K_free, z_free, wn_free, td_free)
H_arx = model_H(f_model, K_static, Z_PAPER, WN_PAPER, TD_PAPER)

# Data points (coherence > 0.1)
coh_thresh = 0.1
pmask = fmask & (coh > coh_thresh)
f_plot = freqs[pmask]
H1_plot = H1[pmask]
coh_plot = coh[pmask]

fig, axes = plt.subplots(3, 1, figsize=(10, 11), gridspec_kw={'height_ratios': [3, 3, 1.5]})
fig.suptitle('Motor Transfer Function Bode Plot  (2026-05-05, 01_leveled_marker)\n'
             r'$H(s) = \frac{K\omega_n^2}{s^2+2\zeta\omega_n s+\omega_n^2}\, e^{-\tau_d s}$',
             fontsize=12, fontweight='bold')

# ── Magnitude ──
ax = axes[0]
mag_data = 20*np.log10(np.abs(H1_plot))
mag_csd = 20*np.log10(np.abs(H_csd))
mag_free = 20*np.log10(np.abs(H_free))
mag_arx = 20*np.log10(np.abs(H_arx))

ax.semilogx(f_plot, mag_data, 'o', color='steelblue', markersize=3, alpha=0.5, label='CSD data (H1)')
ax.semilogx(f_model, mag_csd, '-', color='red', linewidth=2.0,
            label=f'CSD -3dB fit: ζ={z_fit:.3f}, ωn={wn_fit:.1f}, τd={td_fit*1000:.1f}ms')
ax.semilogx(f_model, mag_free, '--', color='darkgreen', linewidth=1.5,
            label=f'CSD free fit: ζ={z_free:.3f}, ωn={wn_free:.1f}, τd={td_free*1000:.1f}ms')
ax.semilogx(f_model, mag_arx, '-.', color='darkorange', linewidth=1.5,
            label=f'ARX (paper): ζ={Z_PAPER}, ωn={WN_PAPER}, τd={TD_PAPER*1000:.1f}ms')

# -3dB marker
mag_dc = mag_csd[0]
ax.axhline(y=mag_dc - 3.0, color='gray', ls='--', alpha=0.4, lw=0.8)
ax.axvline(x=f_3dB, color='gray', ls='--', alpha=0.4, lw=0.8)
ax.plot(f_3dB, mag_dc - 3.0, 'kx', ms=10, mew=2, label=f'−3dB @ {f_3dB:.2f} Hz')

ax.set_ylabel('Magnitude [dB]')
ax.legend(fontsize=7.5, loc='lower left')
ax.grid(True, which='both', alpha=0.3)
ax.set_xlim(f_min, f_max)

# ── Phase ──
ax = axes[1]
ph_data = np.rad2deg(np.unwrap(np.angle(H1_plot)))
ph_csd = np.rad2deg(np.unwrap(np.angle(H_csd)))
ph_free = np.rad2deg(np.unwrap(np.angle(H_free)))
ph_arx = np.rad2deg(np.unwrap(np.angle(H_arx)))

ax.semilogx(f_plot, ph_data, 'o', color='steelblue', markersize=3, alpha=0.5, label='CSD data')
ax.semilogx(f_model, ph_csd, '-', color='red', linewidth=2.0, label='CSD -3dB fit')
ax.semilogx(f_model, ph_free, '--', color='darkgreen', linewidth=1.5, label='CSD free fit')
ax.semilogx(f_model, ph_arx, '-.', color='darkorange', linewidth=1.5, label='ARX (paper)')

ax.set_ylabel('Phase [deg]')
ax.legend(fontsize=7.5)
ax.grid(True, which='both', alpha=0.3)
ax.set_xlim(f_min, f_max)

# ── Coherence ──
ax = axes[2]
ax.semilogx(freqs[fmask], coh[fmask], '-', color='green', linewidth=1.0)
ax.axhline(y=0.5, color='orange', ls='--', alpha=0.6, lw=0.8, label='γ²=0.5')
ax.fill_between(freqs[fmask], 0, coh[fmask], alpha=0.15, color='green')
ax.set_ylabel('Coherence γ²')
ax.set_xlabel('Frequency [Hz]')
ax.set_ylim(0, 1.05)
ax.legend(fontsize=8)
ax.grid(True, which='both', alpha=0.3)
ax.set_xlim(f_min, f_max)

# Parameter table
table_text = (
    "─────────────────────────────────────────\n"
    f"  CSD -3dB:  K={K_fit:.4f}  ζ={z_fit:.4f}  ωn={wn_fit:.2f}  τd={td_fit*1000:.2f}ms\n"
    f"  CSD free:  K={K_free:.4f}  ζ={z_free:.4f}  ωn={wn_free:.2f}  τd={td_free*1000:.2f}ms\n"
    f"  ARX paper: K={K_static:.4f}  ζ={Z_PAPER:.4f}  ωn={WN_PAPER:.2f}  τd={TD_PAPER*1000:.2f}ms\n"
    f"  f_{{-3dB}} = {f_3dB:.2f} Hz   (mean γ² = {np.mean(coh[fmask]):.3f})"
)
fig.text(0.50, 0.005, table_text, transform=fig.transFigure,
         fontsize=8, verticalalignment='bottom', horizontalalignment='center',
         fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

plt.tight_layout(rect=[0, 0.09, 1, 0.94])
plt.savefig(OUTPATH, dpi=600, bbox_inches='tight')
print(f"\nSaved {OUTPATH}")
