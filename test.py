import RPi.GPIO as GPIO
import time

STEP = 17
DIR = 18
ENA = 27  # optional

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Enable driver
GPIO.output(ENA, GPIO.LOW)

# Set direction
GPIO.output(DIR, GPIO.HIGH)
time.sleep(0.0002)  # 200 µs delay for TB6600 direction setup

# Step 200 steps
for _ in range(200):
    GPIO.output(STEP, GPIO.HIGH)
    time.sleep(0.001)  # 1 ms HIGH
    GPIO.output(STEP, GPIO.LOW)
    time.sleep(0.001)  # 1 ms LOW

GPIO.cleanup()
