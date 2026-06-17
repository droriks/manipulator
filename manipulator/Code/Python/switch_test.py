from gpiozero import Button
from time import sleep

switch = Button(17, pull_up=True)
while True:
    print(switch.is_pressed)
    sleep(0.2)