import math
import time
    
class PID:
    def __init__(self, K_PID, k, alpha):
        self.kp = K_PID[0]
        self.ki = K_PID[1]
        self.kd = K_PID[2]
        self.k = k
        self.alpha = alpha  # Low pass filter coefficients
        self.last_output_x = 0
        self.last_output_y = 0
        self.last_error_x = 0
        self.integral_x = 0
        self.last_error_y = 0
        self.integral_y = 0
        self.last_time = None
        
        # Added integral windup protection
        self.integral_limit = 10.0
        
        # Added output limits
        self.output_limit = 20.0  # Maximum phi value

    def compute(self, goal, current_value):
        current_time = time.perf_counter()
        if self.last_time is None:
            self.last_time = current_time
            return 0, 0
        
        dt = current_time - self.last_time
        if dt <= 0:  # Prevent division by zero
            return 0, 0
        
        # Calculate the error
        error_x = goal[0] - current_value[0]
        error_y = goal[1] - current_value[1]
        
        # Calculate the integral value with windup protection
        self.integral_x = max(-self.integral_limit, min(self.integral_limit, 
                             self.integral_x + error_x * dt))
        self.integral_y = max(-self.integral_limit, min(self.integral_limit, 
                             self.integral_y + error_y * dt))
        
        # Calculate the derivative
        derivative_x = (error_x - self.last_error_x) / dt
        derivative_y = (error_y - self.last_error_y) / dt
        
        # Calculate PID output
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        
        # Apply a low pass filter
        output_x = self.alpha * output_x + (1 - self.alpha) * self.last_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y
        
        # Calculate theta and phi
        # Check for zero output to avoid atan2(0,0)
        if abs(output_x) < 1e-6 and abs(output_y) < 1e-6:
            theta = 0
            phi = 0
        else:
            theta = math.degrees(math.atan2(output_y, output_x))
            if theta < 0:
                theta += 360
            
            # Calculate phi with limiting
            phi = self.k * math.sqrt(output_x**2 + output_y**2)
            phi = min(phi, self.output_limit)  # Limit maximum phi

        # Update for next iteration
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_output_x = output_x
        self.last_output_y = output_y
        self.last_time = current_time

        return theta, phi
    
    def reset(self):
        """Reset the PID controller state"""
        self.last_output_x = 0
        self.last_output_y = 0
        self.last_error_x = 0
        self.integral_x = 0
        self.last_error_y = 0
        self.integral_y = 0
        self.last_time = None