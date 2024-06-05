import time

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        return output

# Simulate a system to be controlled
class System:
    def __init__(self, initial_value):
        self.value = initial_value

    def update(self, control_input):
        # Simulate the system's response to the control input
        self.value += control_input * 0.1  # Simple proportional response

# Parameters
Kp = 10.0
Ki = 0.1
Kd = 0.05
setpoint = 10.0
initial_value = 0.0

# Initialize PID controller and system
pid = PID(Kp, Ki, Kd, setpoint)
system = System(initial_value)

# Simulation loop
for i in range(100):
    current_value = system.value
    dt = 0.1  # Time step
    control_input = pid.update(current_value, dt)
    system.update(control_input)

    print(f"Time: {i*dt:.1f}s, System Value: {system.value:.2f}, Control Input: {control_input:.2f}")

    time.sleep(dt)
