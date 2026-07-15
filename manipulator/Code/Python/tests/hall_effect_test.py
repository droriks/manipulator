from gpiozero import DigitalInputDevice
from signal import pause

sensor = DigitalInputDevice(16, pull_up=True)

sensor.when_activated = lambda: print("magnet detected")
sensor.when_deactivated = lambda: print("magnet removed")

pause()  # keep the script alive; gpiozero fires callbacks in the background