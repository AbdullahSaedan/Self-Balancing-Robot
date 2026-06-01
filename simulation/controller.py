class PIDController:
    def __init__(self, Kp, Ki, Kd,
                 dt=0.01,
                 output_limits=(-5.0, 5.0),
                 setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.setpoint = setpoint
        self.output_min, self.output_max = output_limits

        # Internal state
        self._integral = 0.0
        self._prev_measurement = None

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = None

    def compute(self, measurement):
        error = self.setpoint - measurement

        # Proportional term
        P = self.Kp * error

        # Integral term with anti-windup clamping
        self._integral += error * self.dt
        I = self.Ki * self._integral

        # Derivative on measurement (not error) — avoids derivative kick
        if self._prev_measurement is None:
            D = 0.0
        else:
            D = -self.Kd * (measurement - self._prev_measurement) / self.dt

        self._prev_measurement = measurement

        # Raw output
        u_raw = P + I + D

        # Clamp to actuator limits
        u = max(self.output_min, min(self.output_max, u_raw))

        # Anti-windup: back-calculate integral if output is saturated
        if u != u_raw and self.Ki != 0:
            self._integral -= (u_raw - u) / self.Ki

        return u, {'P': P, 'I': I, 'D': D, 'u_raw': u_raw}