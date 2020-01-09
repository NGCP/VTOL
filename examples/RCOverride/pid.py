class PID:
    """PID Controller

    P: Proportional gain, tuning parameter
    I: Integral gain, tuning parameter
    D: Derivative gain, tuning parameter
    dt: Time interval per output

    """

    def __init__(self, P=3, I=1, D=1, dt=0.5):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.dt = dt

        self.clear()


    def clear(self):
        """Clears state variables."""
        self.previous_error = 0
        self.current_derivative = 0
        self.integral = 0


    def update(self, error):
        """Updates state variables based on errors"""
        self.integral = self.integral + error * self.dt
        self.current_derivative = (error - self.previous_error) / self.dt
        self.previous_error = error


    def output(self, error):
        """Outputs control loop feedback"""
        self.update(error)
        output = (self.Kp * error +
                self.Ki * self.integral +
                self.Kd * self.current_derivative)

        return output
