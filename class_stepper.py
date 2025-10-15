import RPi.GPIO as GPIO
import time

class StepperMotor:
    def __init__(self, step_pin, dir_pin, ena_pin, steps_per_rev=200):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.ena_pin = ena_pin
        self.steps_per_rev = steps_per_rev
        self.current_angle = 0.0
        self.angle_per_step = 360.0 / steps_per_rev

        # Set GPIO mode only if not already set
        try:
            GPIO.setmode(GPIO.BCM)
        except RuntimeError:
            pass  # GPIO mode already set

        GPIO.setup(self.ena_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        
        # Initialize pins to known state
        GPIO.output(self.ena_pin, GPIO.HIGH)
        GPIO.output(self.step_pin, GPIO.LOW)
        GPIO.output(self.dir_pin, GPIO.LOW)

    def rotate_to(self, target_angle, delay=0.001):
        try:
            # Calculate shortest rotation
            delta_angle = target_angle - self.current_angle
            delta_angle = (delta_angle + 180) % 360 - 180  # Normalize [-180, 180)

            steps = int(abs(delta_angle) / self.angle_per_step)
            if steps == 0:
                return  # No movement needed
                
            direction = delta_angle > 0

            GPIO.output(self.dir_pin, direction)
            time.sleep(0.0001)  # Small delay for direction setup

            for _ in range(steps):
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(delay)
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(delay)

            # Update current angle, ensuring it stays in [0, 360) range
            self.current_angle = (self.current_angle + delta_angle) % 360
            
        except Exception as e:
            print(f"Stepper motor error on pins {self.step_pin}/{self.dir_pin}: {e}")

    def cleanup(self):
        """Clean up GPIO pins for this motor"""
        try:
            GPIO.output(self.step_pin, GPIO.LOW)
            GPIO.output(self.dir_pin, GPIO.LOW)
            GPIO.output(self.ena_pin, GPIO.LOW)
            GPIO.setup(self.step_pin, GPIO.IN)
            GPIO.setup(self.dir_pin, GPIO.IN)
            GPIO.setup(self.ena_pin, GPIO.IN)
        except Exception as e:
            print(f"Error cleaning up stepper motor: {e}")
            
    def get_current_angle(self):
        """Get the current angle of the motor"""
        return self.current_angle
        
    def set_current_angle(self, angle):
        """Manually set the current angle (for calibration)"""
        self.current_angle = angle % 360
