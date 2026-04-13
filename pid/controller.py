"""
PID Controller Implementation
------------------------------
A clean, well-documented discrete-time PID controller with:
  - Anti-windup (integral clamping)
  - Derivative-on-measurement (avoids derivative kick on setpoint change)
  - Output saturation
"""

class PIDController:
    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        setpoint: float = 0.0,
        output_limits: tuple = (None, None),
        sample_time: float = 0.1,
        anti_windup: bool = True,
        derivative_on_measurement: bool = True,
    ):
        """
        Parameters
        ----------
        Kp : Proportional gain
        Ki : Integral gain
        Kd : Derivative gain
        setpoint : Desired target value
        output_limits : (min, max) tuple to clamp the controller output
        sample_time : dt — time step between calls (seconds)
        anti_windup : Clamp integral term to prevent windup
        derivative_on_measurement : Use -d(measurement)/dt instead of d(error)/dt
                                    Avoids derivative spikes on setpoint changes
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.sample_time = sample_time
        self.anti_windup = anti_windup
        self.derivative_on_measurement = derivative_on_measurement

        # Internal state
        self._integral = 0.0
        self._prev_measurement = None
        self._prev_error = 0.0

        # For logging/debugging
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def reset(self):
        """Reset controller state (use when restarting a simulation)."""
        self._integral = 0.0
        self._prev_measurement = None
        self._prev_error = 0.0
        self.last_p = self.last_i = self.last_d = 0.0

    def compute(self, measurement: float, dt: float = None) -> float:
        """
        Compute the PID output given the current measurement.

        Parameters
        ----------
        measurement : Current process variable reading
        dt : Optional override for time step

        Returns
        -------
        output : Control signal to send to the plant
        """
        if dt is None:
            dt = self.sample_time

        error = self.setpoint - measurement

        # --- Proportional term ---
        P = self.Kp * error

        # --- Integral term with anti-windup ---
        self._integral += error * dt
        I = self.Ki * self._integral

        if self.anti_windup and self.output_limits != (None, None):
            lo, hi = self.output_limits
            if hi is not None:
                self._integral = min(self._integral, hi / (self.Ki + 1e-10))
            if lo is not None:
                self._integral = max(self._integral, lo / (self.Ki + 1e-10))

        # --- Derivative term ---
        if self.derivative_on_measurement:
            # Derivative on measurement: smoother, no kick on setpoint jump
            if self._prev_measurement is None:
                d_term = 0.0
            else:
                d_term = -(measurement - self._prev_measurement) / (dt + 1e-10)
        else:
            # Derivative on error: simpler, but spikes on setpoint change
            d_term = (error - self._prev_error) / (dt + 1e-10)

        D = self.Kd * d_term

        # --- Total output ---
        output = P + I + D

        # --- Clamp output ---
        lo, hi = self.output_limits
        if hi is not None:
            output = min(output, hi)
        if lo is not None:
            output = max(output, lo)

        # --- Save state ---
        self._prev_measurement = measurement
        self._prev_error = error
        self.last_p, self.last_i, self.last_d = P, I, D

        return output

    def __repr__(self):
        return f"PIDController(Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd})"
