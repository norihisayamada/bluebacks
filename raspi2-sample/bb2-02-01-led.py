import RPi.GPIO as GPIO
from time import sleep

LED = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED, GPIO.OUT)

try:
    while True:
        GPIO.output(LED, GPIO.HIGH)
        sleep(0.5)
        GPIO.output(LED, GPIO.LOW)
        sleep(0.5)

except KeyboardInterrupt:
    pass

GPIO.cleanup()
