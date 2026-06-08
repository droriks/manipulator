#Import Libraries\
from gpiozero import AngularServo
from time import sleep
servo = AngularServo(14, min_angle = 0, max_angle = 270, min_pulse_width = .5/1000, max_pulse_width = 2.5/1000)

def set_angle(angle):
    servo.angle = angle
    sleep(1)

try:
    while True:
        angle = int(input("Enter angle (0 to 270): "))
        set_angle(angle)

except KeyboardInterrupt:
    print("Program stopped by user")

