#!/usr/bin/env python3
"""Manual test for the roll servo (PCA9685 channel 8). No ROS."""

from adafruit_pca9685 import PCA9685
import board
import busio

CHANNEL = 2
MIN_PULSE = 500    # us
MAX_PULSE = 2500   # us

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

def set_angle(angle):
    angle = max(0.0, min(180.0, angle))  # clamp
    pulse_us = MIN_PULSE + angle / 180 * (MAX_PULSE - MIN_PULSE)
    pca.channels[CHANNEL].duty_cycle = int(pulse_us / 20000 * 65535)
    print(f"-> {angle:.1f} deg ({pulse_us:.0f} us)")

try:
    while True:
        val = input("Angle (0-180, q to quit): ").strip()
        if val.lower() == 'q':
            break
        try:
            set_angle(float(val))
        except ValueError:
            print("Not a number")
finally:
    pca.channels[CHANNEL].duty_cycle = 0  # go limp on exit
    pca.deinit()