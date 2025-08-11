import class_stepper as cs
import math
import time

class BBrobot:
    def __init__(self, pins):  # Accepts a list of (step_pin, dir_pin) tuples
        # Stepper motors (instead of servos)
        self.s1 = cs.StepperMotor(*pins[0])
        self.s2 = cs.StepperMotor(*pins[1])
        self.s3 = cs.StepperMotor(*pins[2])

        # Link lengths: [base, lower, upper, end-effector]
        self.L = [0.04, 0.04, 0.065, 0.065]
        self.ini_pos = [0, -20, 0.0632] #taking in account the resting position of the motors.
        self.pz_max = 0.0732
        self.pz_min = 0.0532
        self.phi_max = 20

    def set_up(self):
        # No torque setup needed for steppers
        pass

    def clean_up(self):
        # Cleanup GPIO (optional)
        self.s1.cleanup()
        self.s2.cleanup()
        self.s3.cleanup()

    def kinema_inv(self, n, Pz):
        """
        Inverse kinematics calculation with error handling
        """
        try:
            L = self.L
            A = (L[0]+L[1])/Pz
            B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
            C = A**2+1
            D = 2*(A*B-(L[0]+L[1]))
            E = B**2+(L[0]+L[1])**2-L[2]**2
            
            # Check for valid solution
            discriminant = D**2-4*C*E
            if discriminant < 0:
                print("Warning: No valid kinematic solution")
                return [0, 0, 0]
                
            Pmx = (-D+math.sqrt(discriminant))/(2*C)
            
            # Check for valid sqrt
            sqrt_arg = L[2]**2-Pmx**2+2*(L[0]+L[1])*Pmx-(L[0]+L[1])**2
            if sqrt_arg < 0:
                print("Warning: Invalid kinematic solution for Pmz")
                return [0, 0, 0]
                
            Pmz = math.sqrt(sqrt_arg)

            # Joint a
            norm_factor = math.sqrt(n[0]**2 + n[2]**2)
            if norm_factor < 1e-6:
                a_m_x = 0
                a_m_z = Pz
            else:
                a_m_x = (L[3]/norm_factor) * n[2]
                a_m_z = Pz + (L[3]/norm_factor) * (-n[0])
                
            if abs(a_m_z) < 1e-6:
                theta_a = 0
            else:
                A = (L[0]-a_m_x)/a_m_z
                B = (a_m_x**2 + a_m_z**2 - L[2]**2 - L[0]**2 + L[1]**2) / (2*a_m_z)
                C = A**2 + 1
                D = 2*(A*B - L[0])
                E = B**2 + L[0]**2 - L[1]**2
                
                discriminant = D**2 - 4*C*E
                if discriminant < 0:
                    theta_a = 0
                else:
                    ax = (-D + math.sqrt(discriminant)) / (2*C)
                    sqrt_arg = L[1]**2 - ax**2 + 2*L[0]*ax - L[0]**2
                    if sqrt_arg < 0:
                        theta_a = 0
                    else:
                        az = math.sqrt(sqrt_arg)
                        if a_m_z < Pmz:
                            az = -az
                        theta_a = 90 - math.degrees(math.atan2(ax - L[0], az))

            # Joint b
            denom = math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 + 2*math.sqrt(3)*n[0]*n[1])
            if denom < 1e-6:
                theta_b = 0
            else:
                b_m_x = (L[3]/denom)*(-n[2])
                b_m_y = (L[3]/denom)*(-math.sqrt(3)*n[2])
                b_m_z = Pz + (L[3]/denom)*(math.sqrt(3)*n[1]+n[0])
                
                if abs(b_m_z) < 1e-6:
                    theta_b = 0
                else:
                    A = -(b_m_x + math.sqrt(3)*b_m_y + 2*L[0]) / b_m_z
                    B = (b_m_x**2 + b_m_y**2 + b_m_z**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2*b_m_z)
                    C = A**2 + 4
                    D = 2*A*B + 4*L[0]
                    E = B**2 + L[0]**2 - L[1]**2
                    
                    discriminant = D**2 - 4*C*E
                    if discriminant < 0:
                        theta_b = 0
                    else:
                        x = (-D - math.sqrt(discriminant)) / (2*C)
                        y = math.sqrt(3)*x
                        sqrt_arg = L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2
                        if sqrt_arg < 0:
                            theta_b = 0
                        else:
                            z = math.sqrt(sqrt_arg)
                            if b_m_z < Pmz:
                                z = -z
                            theta_b = 90 - math.degrees(math.atan2(math.sqrt(x**2 + y**2) - L[0], z))

            # Joint c
            denom = math.sqrt(n[0]**2 + 3*n[1]**2 + 4*n[2]**2 - 2*math.sqrt(3)*n[0]*n[1])
            if denom < 1e-6:
                theta_c = 0
            else:
                c_m_x = (L[3]/denom)*(-n[2])
                c_m_y = (L[3]/denom)*(math.sqrt(3)*n[2])
                c_m_z = Pz + (L[3]/denom)*(-math.sqrt(3)*n[1]+n[0])
                
                if abs(c_m_z) < 1e-6:
                    theta_c = 0
                else:
                    A = -(c_m_x - math.sqrt(3)*c_m_y + 2*L[0]) / c_m_z
                    B = (c_m_x**2 + c_m_y**2 + c_m_z**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2*c_m_z)
                    C = A**2 + 4
                    D = 2*A*B + 4*L[0]
                    E = B**2 + L[0]**2 - L[1]**2
                    
                    discriminant = D**2 - 4*C*E
                    if discriminant < 0:
                        theta_c = 0
                    else:
                        x = (-D - math.sqrt(discriminant)) / (2*C)
                        y = -math.sqrt(3)*x
                        sqrt_arg = L[1]**2 - 4*x**2 - 4*L[0]*x - L[0]**2
                        if sqrt_arg < 0:
                            theta_c = 0
                        else:
                            z = math.sqrt(sqrt_arg)
                            if c_m_z < Pmz:
                                z = -z
                            theta_c = 90 - math.degrees(math.atan2(math.sqrt(x**2 + y**2) - L[0], z))

            return [theta_a, theta_b, theta_c]
            
        except Exception as e:
            print(f"Kinematic calculation error: {e}")
            return [0, 0, 0]

    def control_t_posture(self, pos, t):
        try:
            theta, phi, Pz = pos
            if phi > self.phi_max:
                phi = self.phi_max
            Pz = min(max(Pz, self.pz_min), self.pz_max)

            z = math.cos(math.radians(phi))
            r = math.sin(math.radians(phi))
            x = r * math.cos(math.radians(theta))
            y = r * math.sin(math.radians(theta))
            n = [x, y, z]

            angles = self.kinema_inv(n, Pz)
            
            # Clamp angles to reasonable limits
            angles = [max(-90, min(90, angle)) for angle in angles]
            
            self.s1.rotate_to(angles[0])
            self.s2.rotate_to(angles[1])
            self.s3.rotate_to(angles[2])
            
            if t > 0:
                time.sleep(t)
                
        except Exception as e:
            print(f"Posture control error: {e}")

    def Initialize_posture(self):
        print("Initializing robot posture...")
        self.control_t_posture(self.ini_pos, t=1)
        print("Robot initialized.")