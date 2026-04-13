"""
Plant Models (Physical Systems)
---------------------------------
Each class represents a real-world system being controlled.
The PID controller sends a control signal; the plant evolves its state in response.

Systems included:
  1. WaterTank   — water level controlled by inlet valve
  2. ThermalRoom — room temperature controlled by heater/cooler
  3. DCMotor     — angular velocity controlled by voltage input
"""

import math


class WaterTank:
    """
    First-order water level system.

    Physics:
        dh/dt = (Q_in - Q_out) / A

    Where:
        h      = water level (m)
        Q_in   = inlet flow rate (m³/s) — this is our control input
        Q_out  = outlet flow rate = Cd * sqrt(h)  (gravity drain)
        A      = tank cross-section area (m²)
        Cd     = discharge coefficient

    Setpoint: Maintain h = target_level metres.
    """

    def __init__(self, area: float = 1.0, discharge_coeff: float = 0.5, initial_level: float = 0.0):
        self.area = area
        self.discharge_coeff = discharge_coeff
        self.level = initial_level

    def step(self, control_input: float, dt: float) -> float:
        """
        Advance the simulation by dt seconds.
        control_input = Q_in (m³/s), clamped to >= 0
        Returns current level (m).
        """
        Q_in = max(0.0, control_input)
        Q_out = self.discharge_coeff * math.sqrt(max(self.level, 0.0))
        dh = (Q_in - Q_out) / self.area
        self.level += dh * dt
        self.level = max(0.0, self.level)  # can't go below zero
        return self.level

    @property
    def state(self):
        return self.level

    def reset(self, initial_level: float = 0.0):
        self.level = initial_level


class ThermalRoom:
    """
    Second-order thermal model (room + wall thermal mass).

    Physics (simplified Newton's law of cooling + heater):
        dT_room/dt = (Q_heater - UA * (T_room - T_outside)) / C_room

    Where:
        T_room    = room temperature (°C)
        Q_heater  = heater output (W) — control input
        UA        = overall heat loss coefficient (W/°C)
        T_outside = ambient outside temperature (°C)
        C_room    = thermal capacity of room (J/°C)

    Setpoint: Maintain T_room = target_temperature °C.
    """

    def __init__(
        self,
        thermal_capacity: float = 5000.0,   # J/°C  (a small room)
        heat_loss_coeff: float = 50.0,       # W/°C
        outside_temp: float = 5.0,           # °C  (cold winter day)
        initial_temp: float = 5.0,           # °C
    ):
        self.C = thermal_capacity
        self.UA = heat_loss_coeff
        self.T_outside = outside_temp
        self.T_room = initial_temp

    def step(self, control_input: float, dt: float) -> float:
        """
        Advance the simulation by dt seconds.
        control_input = heater power (W), positive = heat, negative = cooling
        Returns current room temperature (°C).
        """
        Q_heater = control_input
        dT = (Q_heater - self.UA * (self.T_room - self.T_outside)) / self.C
        self.T_room += dT * dt
        return self.T_room

    @property
    def state(self):
        return self.T_room

    def reset(self, initial_temp: float = 5.0):
        self.T_room = initial_temp


class DCMotor:
    """
    DC Motor angular velocity (speed) control.

    Physics (simplified electrical + mechanical model):
        J * dω/dt = K_t * I - B * ω
        L * dI/dt = V - R * I - K_e * ω

    Simplified (assuming fast electrical dynamics, L ≈ 0):
        I ≈ (V - K_e * ω) / R
        J * dω/dt = K_t * (V - K_e * ω) / R - B * ω

    Where:
        ω   = angular velocity (rad/s)
        V   = input voltage (V) — control input
        K_t = torque constant (N·m/A)
        K_e = back-EMF constant (V·s/rad)
        R   = armature resistance (Ω)
        J   = rotor inertia (kg·m²)
        B   = viscous friction (N·m·s/rad)

    Setpoint: Maintain ω = target_speed rad/s.
    """

    def __init__(
        self,
        J: float = 0.01,    # kg·m²
        B: float = 0.1,     # N·m·s/rad
        K_t: float = 0.5,   # N·m/A
        K_e: float = 0.5,   # V·s/rad
        R: float = 1.0,     # Ω
        initial_speed: float = 0.0,
    ):
        self.J = J
        self.B = B
        self.K_t = K_t
        self.K_e = K_e
        self.R = R
        self.omega = initial_speed

    def step(self, control_input: float, dt: float) -> float:
        """
        Advance the simulation by dt seconds.
        control_input = voltage V (clamped to ±24V)
        Returns current angular velocity (rad/s).
        """
        V = max(-24.0, min(24.0, control_input))
        domega = (self.K_t * (V - self.K_e * self.omega) / self.R - self.B * self.omega) / self.J
        self.omega += domega * dt
        return self.omega

    @property
    def state(self):
        return self.omega

    def reset(self, initial_speed: float = 0.0):
        self.omega = initial_speed
