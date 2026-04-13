"""
Simulation Runner
------------------
Runs a closed-loop simulation of a PID controller + plant model.
Returns a SimResult dataclass with all time-series data for plotting.
"""

from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class SimResult:
    """Holds the complete time-series output of one simulation run."""
    label: str
    time: np.ndarray
    setpoint: np.ndarray
    output: np.ndarray          # process variable (what the sensor measures)
    control: np.ndarray         # controller output (valve position, heater power, voltage)
    p_term: np.ndarray
    i_term: np.ndarray
    d_term: np.ndarray
    Kp: float
    Ki: float
    Kd: float

    @property
    def steady_state_error(self) -> float:
        """Average error in the final 10% of simulation."""
        tail = int(0.9 * len(self.output))
        return float(np.mean(np.abs(self.setpoint[tail:] - self.output[tail:])))

    @property
    def overshoot_pct(self) -> float:
        """Peak overshoot as percentage of setpoint change."""
        sp_change = self.setpoint[-1] - self.output[0]
        if abs(sp_change) < 1e-9:
            return 0.0
        peak = np.max(self.output) if sp_change > 0 else np.min(self.output)
        overshoot = (peak - self.setpoint[-1]) / abs(sp_change) * 100
        return max(0.0, float(overshoot))

    @property
    def rise_time(self) -> Optional[float]:
        """Time to first reach 90% of setpoint (seconds)."""
        sp_final = self.setpoint[-1]
        sp_init = self.output[0]
        target = sp_init + 0.9 * (sp_final - sp_init)
        indices = np.where(self.output >= target)[0] if sp_final > sp_init else np.where(self.output <= target)[0]
        if len(indices) == 0:
            return None
        return float(self.time[indices[0]])

    @property
    def settling_time(self) -> Optional[float]:
        """Time to stay within ±2% band of setpoint (seconds)."""
        sp = self.setpoint[-1]
        band = 0.02 * abs(sp - self.output[0]) if abs(sp - self.output[0]) > 1e-9 else 0.02
        within_band = np.abs(self.output - sp) < band
        # Find last time it exits the band
        outside = np.where(~within_band)[0]
        if len(outside) == 0:
            return 0.0
        last_outside = outside[-1]
        if last_outside + 1 >= len(self.time):
            return None  # Never settled
        return float(self.time[last_outside + 1])


def run_simulation(
    controller,
    plant,
    duration: float,
    dt: float,
    setpoint: float,
    initial_value: float = 0.0,
    disturbance_time: Optional[float] = None,
    disturbance_magnitude: float = 0.0,
    label: str = "Simulation",
) -> SimResult:
    """
    Run a closed-loop PID simulation.

    Parameters
    ----------
    controller   : PIDController instance
    plant        : Plant model (WaterTank, ThermalRoom, DCMotor, etc.)
    duration     : Total simulation time (seconds)
    dt           : Time step (seconds)
    setpoint     : Target value for the controller
    initial_value: Starting state of the plant
    disturbance_time : If set, apply a step disturbance at this time
    disturbance_magnitude : Size of disturbance to inject
    label        : Name for this run (used in plot legends)

    Returns
    -------
    SimResult with all time-series data
    """
    n_steps = int(duration / dt)
    time = np.linspace(0, duration, n_steps)

    # Pre-allocate arrays (faster than appending)
    output_arr  = np.zeros(n_steps)
    control_arr = np.zeros(n_steps)
    setpt_arr   = np.full(n_steps, setpoint)
    p_arr       = np.zeros(n_steps)
    i_arr       = np.zeros(n_steps)
    d_arr       = np.zeros(n_steps)

    # Reset both controller and plant
    controller.reset()
    controller.setpoint = setpoint
    plant.reset(initial_value)

    current_value = initial_value

    for i in range(n_steps):
        t = time[i]

        # Optional: inject disturbance
        if disturbance_time is not None and abs(t - disturbance_time) < dt:
            plant.level = getattr(plant, 'level', current_value) + disturbance_magnitude \
                if hasattr(plant, 'level') else current_value
            # Generic disturbance injection via attribute
            for attr in ('level', 'T_room', 'omega'):
                if hasattr(plant, attr):
                    setattr(plant, attr, getattr(plant, attr) + disturbance_magnitude)
                    break

        # Read current plant state
        current_value = plant.state

        # Compute PID output
        control_signal = controller.compute(current_value, dt)

        # Step plant forward
        plant.step(control_signal, dt)

        # Log
        output_arr[i]  = current_value
        control_arr[i] = control_signal
        setpt_arr[i]   = setpoint
        p_arr[i]       = controller.last_p
        i_arr[i]       = controller.last_i
        d_arr[i]       = controller.last_d

    return SimResult(
        label=label,
        time=time,
        setpoint=setpt_arr,
        output=output_arr,
        control=control_arr,
        p_term=p_arr,
        i_term=i_arr,
        d_term=d_arr,
        Kp=controller.Kp,
        Ki=controller.Ki,
        Kd=controller.Kd,
    )
