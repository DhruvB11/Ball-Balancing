# Ball Balancing Robot (BBRobot)

This project implements a ball balancing robot using a Raspberry Pi, stepper motors, and a camera-based vision system.
The robot detects a ball's position on a platform and dynamically adjusts the platform's tilt to keep the ball at the center using PID control.

## Overview

The system consists of:

* **Camera Module** (`class_Camera.py`)
  Detects the ball position using OpenCV (HoughCircles or contour-based detection) and normalizes coordinates.

* **Ball Balancing Robot Kinematics** (`class_BBRobot.py`)
  Performs inverse kinematics for a 3-legged stepper-motor-controlled platform.

* **Stepper Motor Control** (`class_stepper.py`)
  Controls stepper motors via GPIO pins for precise angular positioning.

* **PID Controller** (`class_PID.py`)
  Calculates tilt angles (`theta`, `phi`) based on ball position errors.

* **Main Control Script** (`main.py`)
  Integrates camera detection, PID computation, and robot motion control in a loop.

---

## Hardware Requirements

* Raspberry Pi (with GPIO support)
* Raspberry Pi Camera (compatible with `picamera2`)
* 3 × Stepper Motors (e.g., NEMA 17)
* Stepper Motor Drivers (e.g., A4988/DRV8825)
* Ball Platform (tri-legged mount)
* Ball (for balancing)
* Jumper Wires & Power Supply

---

## File Structure

```
├── class_BBRobot.py    # Inverse kinematics and posture control
├── class_Camera.py     # Camera capture & ball detection
├── class_PID.py        # PID controller for tilt control
├── class_stepper.py    # Stepper motor control with GPIO
├── main.py             # Main execution script
```

---

## How It Works

1. **Ball Detection**
   The `Camera` class captures frames and detects the ball position. Coordinates are normalized to range `[-1, 1]` in both X and Y axes.

2. **PID Computation**
   The `PID` class calculates the required tilt angle (`phi`) and direction (`theta`) to move the ball towards the center.

3. **Inverse Kinematics**
   The `BBrobot` class converts tilt angles into individual stepper motor positions.

4. **Motor Control**
   The `StepperMotor` class sends step pulses to each motor driver to adjust the platform.

5. **Control Loop**
   The `main.py` script runs continuously, reading camera input, computing control commands, and updating motor positions.

---

## Installation & Usage

### 1. Install Dependencies

```bash
sudo apt update
sudo apt install python3-opencv python3-picamera2 python3-rpi.gpio
```

### 2. Connect Hardware

Wire each stepper motor driver to the Raspberry Pi GPIO pins as defined in `main.py`:

```python
motor_pins = [[17, 18], [22, 23], [24, 25]]
```

### 3. Run the Script

```bash
python3 main.py
```

Press `Ctrl + C` to stop.

---

## PID Tuning

In `main.py`:

```python
K_PID = [0.25, 0.01, 0.005]  # [Kp, Ki, Kd]
k = 1.0                      # Phi scaling factor
alpha = 0.7                   # Low-pass filter coefficient
```

Adjust these values to improve stability and response.

---

## Features

* Two ball detection methods (HoughCircles & contour-based)
* Inverse kinematics with error handling
* Integral windup protection in PID
* Angle & motion clamping to prevent over-rotation
* Camera debug mode for tuning

---

## Notes

* Requires Raspberry Pi OS with camera enabled.
* Works best in a well-lit environment for reliable ball detection.
* Ensure motors are calibrated before running.

---

## License

MIT License. You are free to use and modify this project for personal or academic purposes.
