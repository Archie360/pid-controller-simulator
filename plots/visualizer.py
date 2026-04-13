"""
Visualizer
-----------
All matplotlib plotting lives here. Clean, publication-quality figures.
"""

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from typing import List

from simulation.runner import SimResult

# ── Colour palette ──────────────────────────────────────────────────────────
COLORS = [
    "#2563EB",   # blue
    "#DC2626",   # red
    "#16A34A",   # green
    "#D97706",   # amber
    "#7C3AED",   # violet
]

SETPOINT_COLOR = "#6B7280"   # grey
BAND_ALPHA = 0.08


def _style():
    """Apply a clean, modern matplotlib style."""
    plt.rcParams.update({
        "figure.facecolor":  "#FAFAFA",
        "axes.facecolor":    "#FFFFFF",
        "axes.grid":         True,
        "grid.color":        "#E5E7EB",
        "grid.linewidth":    0.8,
        "axes.spines.top":   False,
        "axes.spines.right": False,
        "font.family":       "DejaVu Sans",
        "font.size":         10,
        "axes.titlesize":    12,
        "axes.titleweight":  "bold",
        "axes.labelsize":    10,
        "legend.framealpha": 0.9,
        "legend.edgecolor":  "#E5E7EB",
    })


# ── Plot 1: Tuning comparison (multiple runs on one axes) ────────────────────

def plot_tuning_comparison(
    results: List[SimResult],
    title: str = "PID Tuning Comparison",
    y_label: str = "Process Variable",
    save_path: str = None,
):
    """
    Show multiple simulation runs on the same axes.
    Perfect for: "here's what happens when you change Kp/Ki/Kd".
    """
    _style()
    fig, (ax_main, ax_ctrl) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]}
    )
    fig.suptitle(title, fontsize=14, fontweight="bold", y=0.98)

    for i, res in enumerate(results):
        color = COLORS[i % len(COLORS)]
        ax_main.plot(res.time, res.output, color=color, linewidth=2.0, label=res.label)
        ax_ctrl.plot(res.time, res.control, color=color, linewidth=1.5, alpha=0.85)

    # Setpoint line (use first result's setpoint)
    sp = results[0].setpoint
    ax_main.plot(results[0].time, sp, color=SETPOINT_COLOR,
                 linewidth=1.5, linestyle="--", label="Setpoint", zorder=0)
    ax_main.fill_between(
        results[0].time,
        sp * 0.98, sp * 1.02,
        color=SETPOINT_COLOR, alpha=BAND_ALPHA, label="±2% band"
    )

    ax_main.set_ylabel(y_label)
    ax_main.legend(loc="lower right")
    ax_main.set_title("System Response")

    ax_ctrl.set_xlabel("Time (s)")
    ax_ctrl.set_ylabel("Control Output")
    ax_ctrl.set_title("Controller Output")

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Saved → {save_path}")
    return fig


# ── Plot 2: Full diagnostic dashboard for a single run ──────────────────────

def plot_diagnostic_dashboard(
    result: SimResult,
    title: str = None,
    y_label: str = "Process Variable",
    save_path: str = None,
):
    """
    4-panel dashboard for a single simulation:
      • Top-left:  System response (PV vs setpoint)
      • Top-right: PID term breakdown (P, I, D contributions)
      • Bottom-left: Error over time
      • Bottom-right: Control output
    """
    _style()
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle(
        title or f"PID Diagnostic — {result.label}",
        fontsize=14, fontweight="bold", y=0.99
    )

    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.38, wspace=0.32)
    ax1 = fig.add_subplot(gs[0, 0])  # Response
    ax2 = fig.add_subplot(gs[0, 1])  # PID terms
    ax3 = fig.add_subplot(gs[1, 0])  # Error
    ax4 = fig.add_subplot(gs[1, 1])  # Control output

    t   = result.time
    sp  = result.setpoint
    pv  = result.output
    err = sp - pv

    # ── Panel 1: Response ──
    ax1.plot(t, pv, color=COLORS[0], linewidth=2, label="Process Variable")
    ax1.plot(t, sp, color=SETPOINT_COLOR, linestyle="--", linewidth=1.5, label="Setpoint")
    ax1.fill_between(t, sp * 0.98, sp * 1.02, color=SETPOINT_COLOR, alpha=BAND_ALPHA)
    ax1.set_title("System Response")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel(y_label)
    ax1.legend()

    # Annotate overshoot
    overshoot = result.overshoot_pct
    if overshoot > 0.5:
        peak_idx = np.argmax(pv) if sp[-1] > pv[0] else np.argmin(pv)
        ax1.annotate(
            f"Overshoot\n{overshoot:.1f}%",
            xy=(t[peak_idx], pv[peak_idx]),
            xytext=(t[peak_idx] + 0.05 * t[-1], pv[peak_idx]),
            arrowprops=dict(arrowstyle="->", color="#DC2626"),
            color="#DC2626", fontsize=9,
        )

    # ── Panel 2: PID term breakdown ──
    ax2.plot(t, result.p_term, color=COLORS[0], linewidth=1.5, label=f"P (Kp={result.Kp})")
    ax2.plot(t, result.i_term, color=COLORS[2], linewidth=1.5, label=f"I (Ki={result.Ki})")
    ax2.plot(t, result.d_term, color=COLORS[1], linewidth=1.5, label=f"D (Kd={result.Kd})")
    ax2.axhline(0, color="#9CA3AF", linewidth=0.8)
    ax2.set_title("PID Term Contributions")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Magnitude")
    ax2.legend()

    # ── Panel 3: Error ──
    ax3.plot(t, err, color=COLORS[3], linewidth=1.8)
    ax3.fill_between(t, err, 0, alpha=0.15, color=COLORS[3])
    ax3.axhline(0, color="#9CA3AF", linewidth=0.8)
    ax3.set_title("Error  (Setpoint − PV)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Error")

    # ── Panel 4: Control output ──
    ax4.plot(t, result.control, color=COLORS[4], linewidth=1.8)
    ax4.axhline(0, color="#9CA3AF", linewidth=0.8)
    ax4.set_title("Controller Output (u)")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Control Signal")

    # Stats box
    rt = result.rise_time
    st = result.settling_time
    stats_text = (
        f"Overshoot:     {overshoot:.1f}%\n"
        f"Rise time:     {'N/A' if rt is None else f'{rt:.2f} s'}\n"
        f"Settling time: {'N/A' if st is None else f'{st:.2f} s'}\n"
        f"Steady-state error: {result.steady_state_error:.4f}"
    )
    fig.text(
        0.5, 0.01, stats_text, ha="center", fontsize=9,
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#EFF6FF", edgecolor="#BFDBFE"),
        family="monospace"
    )

    plt.tight_layout(rect=[0, 0.06, 1, 1])
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Saved → {save_path}")
    return fig


# ── Plot 3: Disturbance rejection ────────────────────────────────────────────

def plot_disturbance_rejection(
    result: SimResult,
    disturbance_time: float,
    y_label: str = "Process Variable",
    save_path: str = None,
):
    """Show how the controller recovers from a sudden disturbance."""
    _style()
    fig, ax = plt.subplots(figsize=(11, 5))
    fig.suptitle(f"Disturbance Rejection — {result.label}", fontsize=13, fontweight="bold")

    ax.plot(result.time, result.output, color=COLORS[0], linewidth=2, label="Process Variable")
    ax.plot(result.time, result.setpoint, color=SETPOINT_COLOR,
            linestyle="--", linewidth=1.5, label="Setpoint")
    ax.fill_between(result.time, result.setpoint * 0.98, result.setpoint * 1.02,
                    color=SETPOINT_COLOR, alpha=BAND_ALPHA)

    ax.axvline(disturbance_time, color=COLORS[1], linestyle=":", linewidth=2,
               label=f"Disturbance at t={disturbance_time}s")
    ax.annotate(
        "← Disturbance injected",
        xy=(disturbance_time, result.setpoint[0]),
        xytext=(disturbance_time + 0.5, result.setpoint[0] * 0.9),
        fontsize=9, color=COLORS[1],
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(y_label)
    ax.legend()
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Saved → {save_path}")
    return fig


# ── Plot 4: Performance summary table ────────────────────────────────────────

def plot_performance_table(results: List[SimResult], save_path: str = None):
    """Render a matplotlib table comparing key metrics across all runs."""
    _style()
    fig, ax = plt.subplots(figsize=(10, len(results) * 0.7 + 1.5))
    ax.axis("off")
    ax.set_title("PID Tuning Performance Summary", fontweight="bold", fontsize=13, pad=15)

    headers = ["Label", "Kp", "Ki", "Kd", "Overshoot (%)", "Rise Time (s)",
               "Settling Time (s)", "SS Error"]
    rows = []
    for r in results:
        st = r.settling_time
        rt = r.rise_time
        rows.append([
            r.label,
            r.Kp, r.Ki, r.Kd,
            f"{r.overshoot_pct:.1f}",
            f"{rt:.2f}" if rt is not None else "—",
            f"{st:.2f}" if st is not None else "—",
            f"{r.steady_state_error:.4f}",
        ])

    table = ax.table(
        cellText=rows,
        colLabels=headers,
        cellLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.6)

    # Style header row
    for j in range(len(headers)):
        table[(0, j)].set_facecolor("#2563EB")
        table[(0, j)].set_text_props(color="white", fontweight="bold")

    # Alternate row shading
    for i in range(1, len(rows) + 1):
        for j in range(len(headers)):
            table[(i, j)].set_facecolor("#EFF6FF" if i % 2 == 0 else "#FFFFFF")

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"  Saved → {save_path}")
    return fig
