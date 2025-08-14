from class_stepper import StepperMotor

stepper1 = StepperMotor()
stepper1.__init__(step_pin=17, dir_pin=27)

print(stepper1.current_angle)