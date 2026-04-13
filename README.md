# PID Controller Simulator

A clean, modular Python simulation of PID controllers applied to three real-world physical systems. Built to demonstrate genuine understanding of feedback control — not just the definition.

---

## What This Demonstrates

A PID (Proportional–Integral–Derivative) controller is the backbone of industrial automation. Every servo motor, thermostat, cruise control, and chemical reactor uses one. This project shows:

- How the **P**, **I**, and **D** terms each contribute to the control response
- What **overshoot**, **oscillation**, and **steady-state error** look like visually
- How **tuning Kp / Ki / Kd** changes system behaviour
- How a well-tuned controller **rejects disturbances** (sudden changes to the system)

---

## Scenarios

| Scenario | System | Control Signal | Setpoint |
|---|---|---|---|
| 🚰 Water Tank | Gravity-drain tank | Inlet flow rate (m³/s) | Water level (m) |
| 🌡️ Thermal Room | Newton's law of cooling | Heater power (W) | Room temperature (°C) |
| ⚙️ DC Motor | Back-EMF motor model | Input voltage (V) | Angular velocity (rad/s) |

---

## Sample Output

Running `python main.py` generates 11 plots in `outputs/`:

### Water Tank — Tuning Comparison
Shows four PID configurations on one chart:
- **P-only**: Sluggish, never quite reaches setpoint (steady-state error)
- **Under-damped**: Oscillates before settling
- **Well-tuned**: Clean approach with minimal overshoot
- **Aggressive**: Fast rise, slight overshoot, quick settling

### 4-Panel Diagnostic Dashboard
For the best-tuned config, shows:
1. Process variable vs setpoint (with overshoot annotation)
2. P / I / D term contributions over time
3. Error signal
4. Raw controller output

### Disturbance Rejection
Injects a sudden disturbance mid-simulation and shows how fast the controller recovers.

### Performance Table
Numerical comparison: overshoot %, rise time, settling time, steady-state error.

---

## File Structure

```
pid-controller-simulator/
│
├── main.py                    # Entry point — run this
│
├── pid/
│   ├── controller.py          # PIDController class (anti-windup, derivative-on-measurement)
│   └── systems.py             # Plant models: WaterTank, ThermalRoom, DCMotor
│
├── simulation/
│   └── runner.py              # run_simulation() + SimResult dataclass
│
├── plots/
│   └── visualizer.py          # All matplotlib plotting functions
│
├── configs/
│   └── params.yaml            # Tuning parameters — edit here to experiment
│
├── outputs/                   # Generated plots land here (git-ignored)
│
└── requirements.txt
```

---

## Quick Start

```bash
# Clone and install
git clone https://github.com/your-username/pid-controller-simulator.git
cd pid-controller-simulator
pip install -r requirements.txt

# Run all three scenarios
python main.py

# Run only one scenario
python main.py --scenario tank
python main.py --scenario room
python main.py --scenario motor

# Show plots interactively instead of saving
python main.py --show
```

---

## The Core PID Loop

```python
error        = setpoint - measurement
proportional = Kp * error
integral    += Ki * error * dt
derivative   = Kd * (error - prev_error) / dt

output = proportional + integral + derivative
```

### What each term does

| Term | Effect | Too high → | Too low → |
|---|---|---|---|
| **Kp** (Proportional) | Immediate response to error | Oscillation / instability | Slow response |
| **Ki** (Integral) | Eliminates steady-state error | Integral windup, overshoot | Persistent offset |
| **Kd** (Derivative) | Predicts future error, damps overshoot | Noise amplification | No damping |

---

## Engineering Details

### Anti-Windup
When the controller output saturates (hits physical limits like a fully open valve), the integral term keeps accumulating — this is *integral windup*. The controller clamps the integral to prevent this.

### Derivative on Measurement
Standard PID computes `d(error)/dt`, which spikes when the setpoint changes suddenly. This implementation uses `-d(measurement)/dt` instead — same result during steady tracking, but no derivative kick on setpoint steps.

### Output Saturation
All control outputs are clamped to physical limits:
- Water tank: flow rate 0–5 m³/s (can't have negative flow)
- Room heater: ±2000 W
- DC motor: ±24 V supply rail

---

## Tuning Guide

Open `configs/params.yaml` and modify the `tuning_configs` section. Add as many configurations as you want — each gets its own line on the comparison plot.

**Ziegler–Nichols starting point:**
1. Set Ki = Kd = 0
2. Increase Kp until the system oscillates steadily → this is Ku (ultimate gain)
3. Measure the oscillation period Tu
4. Set: `Kp = 0.6·Ku`, `Ki = 1.2·Ku/Tu`, `Kd = 0.075·Ku·Tu`

---

## Extending the Project

- **Add a new plant**: Create a class in `pid/systems.py` with `.step(control, dt)` and `.state` property — it plugs straight into `run_simulation()`
- **Add noise**: Inject Gaussian noise into `measurement` inside `runner.py` to simulate sensor noise
- **Add a setpoint profile**: Pass a time-varying setpoint array to `run_simulation()` instead of a scalar
- **Real-time animation**: Use `matplotlib.animation.FuncAnimation` to animate the tank filling live

---

## Requirements

- Python 3.9+
- numpy
- matplotlib
- PyYAML
