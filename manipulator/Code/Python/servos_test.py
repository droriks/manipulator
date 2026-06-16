from adafruit_pca9685 import PCA9685
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

def angle_to_duty(angle):
    # -30 to 150 degree range
    pulse_us = 2500 - (angle + 30) / 180 * (2500 - 500)
    return int(pulse_us / 20000 * 65535)

try:
    while True:
        angle = int(input("Enter angle (-30 to 150): "))
        pca.channels[0].duty_cycle = angle_to_duty(angle)  # change 0 to whichever channel
except KeyboardInterrupt:
    pca.deinit()
    print("Stopped")