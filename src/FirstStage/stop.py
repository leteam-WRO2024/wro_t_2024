from gpiozero import Motor
from time import sleep

print("Test")

motorLeft = Motor(24, 13)
motorLeft.stop()
