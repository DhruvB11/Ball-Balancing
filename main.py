import time
import math
from class_BBRobot import BBrobot
from class_Camera import Camera  # Fixed: Changed from PiCamera3Tracker
from class_PID import PID

# Initialize camera
camera = Camera()  # Fixed: Use correct class name

# Initialize robot
motor_pins = [[17, 18, 27], [22, 23, 5], [24, 25, 6]]  # [step, dir, ena] for each motor
robot = BBrobot(motor_pins)
robot.set_up()

# Initialize PID controller
# Fixed: Use correct PID constructor parameters
K_PID = [0.25, 0.01, 0.005]  # [Kp, Ki, Kd]
k = 1.0  # Scaling factor for phi calculation
alpha = 0.7  # Low pass filter coefficient
pid = PID(K_PID, k, alpha)

# Define target position (center of the platform)
target_position = [320.0, 240.0]

try:
    robot.Initialize_posture()
    time.sleep(1)

    while True:
        # Get ball position
        current_position = camera.get_ball_position()  # Fixed: Use correct method name
        
        # Check if ball is detected
        if current_position == [0, 0]:
            # No ball detected, maintain current position or return to center
            theta, phi = 0, 0
        else:
            # Get PID outputs (theta and phi)
            theta, phi = pid.compute(target_position, current_position)  # Fixed: Pass both goal and current

        # Ensure phi is within limits
        phi = min(phi, robot.phi_max)
        
        # Command robot with current height
        robot.control_t_posture([theta, phi, robot.ini_pos[2]], t=0.05)  # Faster update rate

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    robot.clean_up()
    camera.close()  # Fixed: Use correct method name
