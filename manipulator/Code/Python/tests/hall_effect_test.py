import time
from gpiozero import DigitalInputDevice


sensor = DigitalInputDevice(pin=16, pull_up=True)

while True:
    if magnet_detected:
        print("magnet detected")

    if magnet_removed:
        print("magnet removed")
    time.sleep(.5)

