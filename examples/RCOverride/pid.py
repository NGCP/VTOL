'''Implementation of PID'''
class PID:
    """PID Controller

    P: Proportional gain, tuning parameter
    I: Integral gain, tuning parameter
    D: Derivative gain, tuning parameter
    d_t: Time interval per output

    """

    def __init__(self, P=3, I=1, D=1, d_t=0.5):
        self.k_p = P
        self.k_i = I
        self.k_d = D

        self.d_t = d_t
        self.previous_error = 0
        self.integral = 0
        self.current_derivative = 0


    def clear(self):
        """Clears state variables."""
        self.previous_error = 0
        self.current_derivative = 0
        self.integral = 0


    def update(self, error):
        """Updates state variables based on errors"""
        self.integral = self.integral + error * self.d_t
        self.current_derivative = (error - self.previous_error) / self.d_t
        self.previous_error = error


    def output(self, error):
        """Outputs control loop feedback"""
        self.update(error)
        output = (self.k_p * error + \
                self.k_i * self.integral + \
                self.k_d * self.current_derivative)

        return output
