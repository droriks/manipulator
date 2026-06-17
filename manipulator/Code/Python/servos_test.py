from gpiozero import Button
from adafruit_pca9685 import PCA9685
import board
import busio
from time import sleep

# --- Setup ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

switch = Button(17, pull_up=True)

def angle_to_duty(angle):
    pulse_us = 500 + (angle) / 180 * (2500 - 500)
    return int(pulse_us / 20000 * 65535)

def set_angle(channel, angle):
    pca.channels[channel].duty_cycle = angle_to_duty(angle)

# --- Test ---
angle = 90.0
channel = 0  # change to whichever channel you're testing

set_angle(channel, angle)
sleep(3)

print("Starting — driving down until switch hit")

try:
    while not switch.is_pressed:
        angle += 0.5
        print(f"Angle: {angle:.1f}")
        set_angle(channel, angle)
        sleep(0.05)

    print(f"Switch hit at {angle:.1f} degrees — returning 30 degrees up")
    
    target = angle + 30.0
    while angle < target:
        angle += 0.5
        print(f"Returning: {angle:.1f}")
        set_angle(channel, angle)
        sleep(0.05)

    print(f"Done — stopped at {angle:.1f} degrees")

except KeyboardInterrupt:
    print("Stopped")

finally:
    pca.deinit()