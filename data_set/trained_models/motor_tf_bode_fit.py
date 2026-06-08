#!/usr/bin/env python3
"""
Motor Transfer Function Bode Fitting.

Model: H_rot(s) = ω_{rot,n}² / (s² + 2ζ ω_{rot,n} s + ω_{rot,n}²) · exp(-τ_d s)

Constraints: K=1 (DC gain), f_{-3dB}=3.1 Hz
Free parameters: ζ, τ_d  (ω_{rot,n} derived from -3dB constraint)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution

OUTPATH = '/home/user/drone_simulator_pkgs/data_set/trained_models/motor_tf_bode_comparison.png'

# Measured Bode data (cmd_raw RPM → actual_rpm)
f_data = np.array([0.8, 2.3, 3.1, 4.7, 7.0, 7.8, 10.2, 14.8, 20.3, 29.7])
mag_db_data = np.array([-0.1, -2.8, -4.4, -7.6, -11.6, -13.2, -17.3, -24.4, -32.3, -43.1])
phase_deg_data = np.array([-18.7, -63.8, -86.2, -123.8, -159.2, -168.8, -196.7, -241.8, -287.8, -346.9])
w_data = 2 * np.pi * f_data

F_3DB = 3.1  # Hz
OMEGA_3DB = 2 * np.pi * F_3DB


def omega_n_from_3dB(zeta, omega_3dB):
    """ωn from -3dB constraint: u = 1-2ζ²+√(4ζ⁴-4ζ²+2), ωn = ω_{-3dB}/√u"""
    u = 1 - 2*zeta**2 + np.sqrt(4*zeta**4 - 4*zeta**2 + 2)
    return omega_3dB / np.sqrt(u) if u > 0 else 1e6


def model_H(w, wn, zeta, tau_d):
    s = 1j * w
    return wn**2 / (s**2 + 2*zeta*wn*s + wn**2) * np.exp(-tau_d * s)


def cost(params):
    zeta, tau_d = params
    if zeta <= 0.1 or tau_d < 0:
        return 1e10
    wn = omega_n_from_3dB(zeta, OMEGA_3DB)
    H = model_H(w_data, wn, zeta, tau_d)
    mag_db = 20 * np.log10(np.abs(H))
    phase_deg = np.unwrap(np.angle(H)) * 180 / np.pi
    err_mag = np.sum((mag_db - mag_db_data)**2)
    err_phase = np.sum(((phase_deg - phase_deg_data) / 10)**2)
    return err_mag + err_phase


# Fit
res = differential_evolution(cost, [(0.3, 2.0), (0.0, 0.1)],
                             seed=42, maxiter=2000, tol=1e-12, polish=True)
zeta, tau_d = res.x
wn = omega_n_from_3dB(zeta, OMEGA_3DB)
fn = wn / (2 * np.pi)

print(f"zeta    = {zeta:.4f}")
print(f"w_rot,n = {wn:.2f} rad/s ({fn:.2f} Hz)")
print(f"tau_d   = {tau_d*1000:.2f} ms")
print(f"f_-3dB  = {F_3DB} Hz (constrained)")

# Model curve
f_model = np.logspace(np.log10(0.3), np.log10(50), 500)
w_model = 2 * np.pi * f_model
H_fit = model_H(w_model, wn, zeta, tau_d)
mag_fit = 20 * np.log10(np.abs(H_fit))
phase_fit = np.unwrap(np.angle(H_fit)) * 180 / np.pi

# -180 deg crossing
idx180 = np.where(phase_fit < -180)[0]
f_180 = None
if len(idx180) > 0:
    i = idx180[0]
    f_a, f_b = f_model[i-1], f_model[i]
    p_a, p_b = phase_fit[i-1], phase_fit[i]
    f_180 = f_a + (f_b - f_a) * (-180 - p_a) / (p_b - p_a)

# Plot
fig, axes = plt.subplots(2, 1, figsize=(8, 8))
fig.suptitle(r'$H_{rot}(s)$', fontsize=14, fontweight='bold')

# Magnitude
ax = axes[0]
ax.semilogx(f_data, mag_db_data, 'ko', markersize=7, label='Measured')
ax.semilogx(f_model, mag_fit, 'b-', linewidth=2.0,
            label=rf'$\zeta={zeta:.4f},\;\omega_{{rot,n}}={wn:.2f}$ rad/s, $\tau_d={tau_d*1000:.2f}$ ms')
ax.axhline(y=-3.0, color='gray', ls='--', alpha=0.5, lw=0.8)
ax.axvline(x=F_3DB, color='gray', ls='--', alpha=0.5, lw=0.8)
ax.plot(F_3DB, -3.0, 'rx', ms=10, mew=2, label=f'$-3$ dB @ {F_3DB} Hz')
ax.set_ylabel('Magnitude [dB]')
ax.legend(fontsize=9)
ax.grid(True, which='both', alpha=0.3)
ax.set_xlim(0.3, 50)
ax.set_ylim(-50, 5)

# Phase
ax = axes[1]
ax.semilogx(f_data, phase_deg_data, 'ko', markersize=7, label='Measured')
ax.semilogx(f_model, phase_fit, 'b-', linewidth=2.0, label='Fit')
ax.axhline(y=-180, color='r', ls=':', alpha=0.5, lw=0.8)
if f_180:
    ax.axvline(x=f_180, color='r', ls=':', alpha=0.5, lw=0.8)
    ax.text(f_180 * 1.1, -185, f'$-180°$ @ {f_180:.1f} Hz', color='r', fontsize=9)
ax.set_ylabel('Phase [deg]')
ax.set_xlabel('Frequency [Hz]')
ax.legend(fontsize=9)
ax.grid(True, which='both', alpha=0.3)
ax.set_xlim(0.3, 50)

plt.tight_layout()
plt.savefig(OUTPATH, dpi=600, bbox_inches='tight')
print(f"Saved {OUTPATH}")
