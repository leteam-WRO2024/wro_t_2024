from gpiozero import Motor
from time import sleep

print("Test")

motorLeft = Motor(13, 24)
motorLeft.stop()
