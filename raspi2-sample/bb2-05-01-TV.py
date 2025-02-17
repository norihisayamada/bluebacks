# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from time import sleep
import subprocess

def my_callback(channel):
    if channel==23:
        args = ['irsend', '-#', '1', 'SEND_ONCE', 'TV', 'power']
        subprocess.Popen(args)
    elif channel==22:
        args = ['irsend', '-#', '1', 'SEND_ONCE', 'TV', 'cup']
        subprocess.Popen(args)
    elif channel==27:
        args = ['irsend', '-#', '1', 'SEND_ONCE', 'TV', 'cdown']
        subprocess.Popen(args)
    elif channel==17:
        args = ['irsend', '-#', '1', 'SEND_ONCE', 'TV', 'vup']
        subprocess.Popen(args)
    elif channel==18:
        args = ['irsend', '-#', '1', 'SEND_ONCE', 'TV', 'vdown']
        subprocess.Popen(args)

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.add_event_detect(23, GPIO.RISING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(22, GPIO.RISING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(27, GPIO.RISING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(17, GPIO.RISING, callback=my_callback, bouncetime=200)
GPIO.add_event_detect(18, GPIO.RISING, callback=my_callback, bouncetime=200)

try:
    while True:
        sleep(1)

except KeyboardInterrupt:
    pass

GPIO.cleanup()

