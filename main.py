"""
PID Controller Simulator — main.py
====================================
Entry point. Runs all three scenario simulations and saves plots to outputs/.

Usage:
    python main.py                    # Run all scenarios
    python main.py --scenario tank    # Run only water tank
    python main.py --scenario room    # Run only thermal room
    python main.py --scenario motor   # Run only DC motor
    python main.py --show             # Display plots interactively (don't save)
"""

import argparse
import os
import sys
import yaml

# Make local modules importable
sys.path.insert(0, os.path.dirname(__file__))

from pid import PIDController, WaterTank, ThermalRoom, DCMotor
from simulation import run_simulation
from plots import (
    plot_tuning_comparison,
    plot_diagnostic_dashboard,
    plot_disturbance_rejection,
    plot_performance_table,
)


# ── Load config ──────────────────────────────────────────────────────────────

def load_config(path: str = "configs/params.yaml") -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


# ── Scenario runners ─────────────────────────────────────────────────────────

def run_water_tank(cfg: dict, output_dir: str, show: bool):
    print("\n🚰  Running: Water Tank Scenario")
    print("   Controlling water level via inlet valve flow rate.")

    sc  = cfg["water_tank"]
    sys_cfg  = sc["system"]
    sim_cfg  = sc["simulation"]
    tunings  = sc["tuning_configs"]

    results = []
    for tune in tunings:
        plant = WaterTank(**sys_cfg)
        ctrl = PIDController(
            Kp=tune["Kp"], Ki=tune["Ki"], Kd=tune["Kd"],
            output_limits=(0.0, 5.0),
            sample_time=sim_cfg["dt"],
        )
        res = run_simulation(
            controller=ctrl,
            plant=plant,
            duration=sim_cfg["duration"],
            dt=sim_cfg["dt"],
            setpoint=sim_cfg["setpoint"],
            initial_value=sys_cfg["initial_level"],
            label=tune["label"],
        )
        results.append(res)
        st = res.settling_time
        print(f"   ✓ {tune['label']:35s}  ",f"overshoot={res.overshoot_pct:5.1f}%  ",f"settling={'N/A' if st is None else f'{st:.1f}s'}")

    # Best-tuned result for the diagnostic dashboard
    best = results[2]  # "Well-tuned" config

    # Disturbance rejection sim (using best tuning)
    plant = WaterTank(**sys_cfg)
    ctrl_best = PIDController(
        Kp=best.Kp, Ki=best.Ki, Kd=best.Kd,
        output_limits=(0.0, 5.0),
        sample_time=sim_cfg["dt"],
    )
    dist_result = run_simulation(
        controller=ctrl_best,
        plant=plant,
        duration=sim_cfg["duration"],
        dt=sim_cfg["dt"],
        setpoint=sim_cfg["setpoint"],
        initial_value=sys_cfg["initial_level"],
        disturbance_time=sim_cfg["disturbance_time"],
        disturbance_magnitude=sim_cfg["disturbance_magnitude"],
        label="Disturbance Test",
    )

    if not show:
        plot_tuning_comparison(
            results, title="Water Tank — PID Tuning Comparison",
            y_label="Water Level (m)",
            save_path=os.path.join(output_dir, "tank_tuning_comparison.png"),
        )
        plot_diagnostic_dashboard(
            best, title="Water Tank — Well-Tuned PID Dashboard",
            y_label="Water Level (m)",
            save_path=os.path.join(output_dir, "tank_diagnostic_dashboard.png"),
        )
        plot_disturbance_rejection(
            dist_result, disturbance_time=sim_cfg["disturbance_time"],
            y_label="Water Level (m)",
            save_path=os.path.join(output_dir, "tank_disturbance_rejection.png"),
        )
        plot_performance_table(
            results,
            save_path=os.path.join(output_dir, "tank_performance_table.png"),
        )
    else:
        import matplotlib.pyplot as plt
        plot_tuning_comparison(results, y_label="Water Level (m)")
        plot_diagnostic_dashboard(best, y_label="Water Level (m)")
        plot_disturbance_rejection(dist_result, sim_cfg["disturbance_time"], y_label="Water Level (m)")
        plot_performance_table(results)
        plt.show()


def run_thermal_room(cfg: dict, output_dir: str, show: bool):
    print("\n🌡️   Running: Thermal Room Scenario")
    print("   Controlling room temperature via heater/cooler output.")

    sc = cfg["thermal_room"]
    sys_cfg  = sc["system"]
    sim_cfg  = sc["simulation"]
    tunings  = sc["tuning_configs"]

    results = []
    for tune in tunings:
        plant = ThermalRoom(**sys_cfg)
        ctrl = PIDController(
            Kp=tune["Kp"], Ki=tune["Ki"], Kd=tune["Kd"],
            output_limits=(-2000.0, 2000.0),   # Heater: ±2 kW
            sample_time=sim_cfg["dt"],
        )
        res = run_simulation(
            controller=ctrl,
            plant=plant,
            duration=sim_cfg["duration"],
            dt=sim_cfg["dt"],
            setpoint=sim_cfg["setpoint"],
            initial_value=sys_cfg["initial_temp"],
            label=tune["label"],
        )
        results.append(res)
        st = res.settling_time
        print(f"   ✓ {tune['label']:30s}  ",f"overshoot={res.overshoot_pct:5.1f}%  ",f"settling={'N/A' if st is None else f'{st:.1f}s'}")

    best = results[1]  # "PID balanced"

    # Disturbance test
    plant = ThermalRoom(**sys_cfg)
    ctrl_best = PIDController(
        Kp=best.Kp, Ki=best.Ki, Kd=best.Kd,
        output_limits=(-2000.0, 2000.0),
        sample_time=sim_cfg["dt"],
    )
    dist_result = run_simulation(
        controller=ctrl_best,
        plant=plant,
        duration=sim_cfg["duration"],
        dt=sim_cfg["dt"],
        setpoint=sim_cfg["setpoint"],
        initial_value=sys_cfg["initial_temp"],
        disturbance_time=sim_cfg["disturbance_time"],
        disturbance_magnitude=sim_cfg["disturbance_magnitude"],
        label="Disturbance Test (cold gust)",
    )

    if not show:
        plot_tuning_comparison(
            results, title="Thermal Room — PID Tuning Comparison",
            y_label="Temperature (°C)",
            save_path=os.path.join(output_dir, "room_tuning_comparison.png"),
        )
        plot_diagnostic_dashboard(
            best, title="Thermal Room — PID Dashboard",
            y_label="Temperature (°C)",
            save_path=os.path.join(output_dir, "room_diagnostic_dashboard.png"),
        )
        plot_disturbance_rejection(
            dist_result, disturbance_time=sim_cfg["disturbance_time"],
            y_label="Temperature (°C)",
            save_path=os.path.join(output_dir, "room_disturbance_rejection.png"),
        )
        plot_performance_table(
            results,
            save_path=os.path.join(output_dir, "room_performance_table.png"),
        )
    else:
        import matplotlib.pyplot as plt
        plot_tuning_comparison(results, y_label="Temperature (°C)")
        plot_diagnostic_dashboard(best, y_label="Temperature (°C)")
        plot_disturbance_rejection(dist_result, sim_cfg["disturbance_time"], y_label="Temperature (°C)")
        plot_performance_table(results)
        plt.show()


def run_dc_motor(cfg: dict, output_dir: str, show: bool):
    print("\n⚙️   Running: DC Motor Speed Control Scenario")
    print("   Controlling angular velocity via input voltage.")

    sc = cfg["dc_motor"]
    sys_cfg  = sc["system"]
    sim_cfg  = sc["simulation"]
    tunings  = sc["tuning_configs"]

    results = []
    for tune in tunings:
        plant = DCMotor(**sys_cfg)
        ctrl = PIDController(
            Kp=tune["Kp"], Ki=tune["Ki"], Kd=tune["Kd"],
            output_limits=(-24.0, 24.0),   # Voltage limits
            sample_time=sim_cfg["dt"],
        )
        res = run_simulation(
            controller=ctrl,
            plant=plant,
            duration=sim_cfg["duration"],
            dt=sim_cfg["dt"],
            setpoint=sim_cfg["setpoint"],
            initial_value=sys_cfg["initial_speed"],
            label=tune["label"],
        )
        results.append(res)
        st = res.settling_time
        print(f"   ✓ {tune['label']:30s}  ",f"settling={'N/A' if st is None else f'{st:.1f}s'}"),f"overshoot={res.overshoot_pct:5.1f}%  "

    best = results[1]

    if not show:
        plot_tuning_comparison(
            results, title="DC Motor — PID Tuning Comparison",
            y_label="Angular Velocity (rad/s)",
            save_path=os.path.join(output_dir, "motor_tuning_comparison.png"),
        )
        plot_diagnostic_dashboard(
            best, title="DC Motor — PID Dashboard",
            y_label="Angular Velocity (rad/s)",
            save_path=os.path.join(output_dir, "motor_diagnostic_dashboard.png"),
        )
        plot_performance_table(
            results,
            save_path=os.path.join(output_dir, "motor_performance_table.png"),
        )
    else:
        import matplotlib.pyplot as plt
        plot_tuning_comparison(results, y_label="Angular Velocity (rad/s)")
        plot_diagnostic_dashboard(best, y_label="Angular Velocity (rad/s)")
        plot_performance_table(results)
        plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="PID Controller Simulator")
    parser.add_argument(
        "--scenario", choices=["tank", "room", "motor", "all"],
        default="all", help="Which scenario to run"
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display plots interactively instead of saving to files"
    )
    parser.add_argument(
        "--config", default="configs/params.yaml",
        help="Path to YAML config file"
    )
    parser.add_argument(
        "--output-dir", default="outputs",
        help="Directory to save plots"
    )
    args = parser.parse_args()

    # Load config
    cfg = load_config(args.config)

    # Create output dir
    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 55)
    print("  PID Controller Simulator")
    print("=" * 55)

    if args.scenario in ("all", "tank"):
        run_water_tank(cfg, args.output_dir, args.show)

    if args.scenario in ("all", "room"):
        run_thermal_room(cfg, args.output_dir, args.show)

    if args.scenario in ("all", "motor"):
        run_dc_motor(cfg, args.output_dir, args.show)

    if not args.show:
        print(f"\n✅  All plots saved to '{args.output_dir}/'")
        files = sorted(os.listdir(args.output_dir))
        for f in files:
            print(f"   {f}")
    print()


if __name__ == "__main__":
    main()
