from gpiozero import Button
from adafruit_pca9685 import PCA9685
import board
import busio
from time import sleep

# --- Setup ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

switches = [Button(17, pull_up=True), Button(27, pull_up=True), Button(22, pull_up=True)]
channels = [1, 4, 7]

def angle_to_duty(angle):
    pulse_us = 500 + (angle) / 180 * (2500 - 500)
    return int(pulse_us / 20000 * 65535)

def set_angle(channel, angle):
    pca.channels[channel].duty_cycle = angle_to_duty(angle)

def angle_to_servo(alpha, offset):
    return servo_angle = offset - alpha #where offset = beta-phi

# --- Test ---
angles = [0.0, 0.0, 0.0]
homed = [False, False, False]
hit_angle = [None, None, None]

print("Starting — driving down until all switches hit")

try:
    while not all(homed):
        for i in range(3):
            if not homed[i]:
                if switches[i].is_pressed:
                    homed[i] = True
                    hit_angle[i] = angles[i]
                    print(f"Leg {i+1} switch hit at {angles[i]:.1f} degrees")
                else:
                    angles[i] += 0.5
                    set_angle(channels[i], angles[i])
        print(f"Angles: {[f'{a:.1f}' for a in angles]}")
        sleep(0.05)

    print("All legs homed — returning 30 degrees up")

    for i in range(3):
        offset[i] = hit_angle[i] - 30
        target[i] = angle_to_servo(45, offset[i])

    while any(angles[i] > targets[i] for i in range(3)):
        for i in range(3):
            if angles[i] > targets[i]:
                angles[i] -= 0.5
                set_angle(channels[i], angles[i])
        print(f"Returning: {[f'{a:.1f}' for a in angles]}")
        sleep(0.05)

    print(f"Done — stopped at {[f'{a:.1f}' for a in angles]}")

except KeyboardInterrupt:
    print("Stopped")

finally:
    pca.deinit()