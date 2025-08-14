import RPi.GPIO as GPIO
import time

STEP_PIN = 17
DIR_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

GPIO.output(DIR_PIN, GPIO.HIGH)  # or LOW

for _ in range(200):
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(0.005)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(0.005)

GPIO.cleanup()
