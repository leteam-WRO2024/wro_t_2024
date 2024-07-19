import serial
from mpu_ctrl import MPU
from gpiozero import Button, LED
from motor_ctrl import mctrl
from time import sleep, time
from threading import Thread, Event
from ColorCoords import ColorsCoordinations
import cv2
import numpy as np

new_J = 0


def read_distance(event: Event):
    global frontDist, rightDist, leftDist
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()

    while not event.is_set():
        if ser.in_waiting > 0:
            frontDist, rightDist, leftDist = list(
                map(float, ser.readline().decode("utf-8").rstrip().split(",")))
            
def mpu_loop(mpu: MPU):
    global new_J
    mpu.setUp()
    mpu.calibrateGyro(500)

    while True:
        try:
            mpu.currentAngle = mpu.compFilter(1) + ((3 * new_J) * motors.direction)
            sleep(0.00005)
        except:
            pass


def get_color(event: Event):
    global green_center, red_center
    motors.direction = 0

    while True:
        if not event.is_set():
            orange = color_reco.detect_color("orange")
            blue = color_reco.detect_color("blue")
            if orange != (-1, -1):
                motors.direction = -1
            elif blue != (-1, -1):
                motors.direction = 1
            else:
                if leftDist - rightDist > 83:
                    motors.direction = 1
                elif rightDist - leftDist > 83:
                    motors.direction = -1

        if motors.direction != 0:
            event.set()

        green_center = color_reco.detect_color("green")
        red_center = color_reco.detect_color("red")
        sleep(0.00005)


def starting_point_adjustment():
    heading = motors.get_heading_angle(mpu.currentAngle)
    motors.move_forward(0.7)

    while frontDist > 140 or frontDist == 0:
        # motors.adjust_angle(heading, mpu.currentAngle, rightDist, leftDist)
        print("Adjusting")
    print("out")

    motors.stop_car()
    motors.turn_forward()




def moveUntilDist():
    heading = motors.get_heading_angle(mpu.currentAngle)

    motors.move_forward()

    motors.minfo(f"\n\nFRONT:: {frontDist}\n\n")
    while True:
        if green_center[2] != -1:
            motors.color_based_adjustment(heading, mpu.currentAngle, green_center, 1, leftDist)
        elif red_center[2] != -1:
            motors.color_based_adjustment(heading, mpu.currentAngle, red_center, -1, leftDist)
            motors.minfo(
                f"\nFRONT:: {frontDist}\nRIGHT:: {rightDist}\nLEFT:: {leftDist}\n")
        else:
            if (frontDist > 75 or frontDist == 0) or (rightDist < 100 and leftDist < 100):
                motors.color_based_adjustment(heading, mpu.currentAngle, green_center, 0, leftDist)
            else:
                break
        
    motors.turn_forward()
    motors.stop_car()
    motors.speed = 0.8
    print("OUT OF LOOP")


def turn():
    heading = motors.get_heading_angle(mpu.currentAngle)

    mpu.nextAngle = motors.get_next_angle(heading, True)
    motors.direction_turn()
    sleep(0.003)
    motors.move_forward(0.8)

    mpu.dinfo(
        f"ABOVE:: ({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")

    while not (250 > (mpu.currentAngle - mpu.nextAngle) * motors.direction >= -50):
        mpu.dinfo(
            f"({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")
        sleep(0.0000005)
    mpu.dinfo(f"UNDER:: ({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")

    motors.stop_car()
    motors.reverse_angle()
    motors.move_backward(0.7)
    while not (10 > ((mpu.currentAngle - motors.get_heading_angle(mpu.nextAngle)) * motors.direction) > -20):
        sleep(0.0001)
    motors.turn_forward()
    motors.stop_car()
    sleep(0.6)


def main():
    global new_J

    motors.max_angle = 120
    motors.min_angle = 40
    motors.mid_angle = 80
    motors.speed = 0.7
    motors.turn_forward()

    color_event = Event()
    dist_event = Event()

    Thread(target=mpu_loop, args=(mpu, )).start()

    ready = mpu.isReady()

    Thread(target=read_distance, args=(dist_event, )).start()
    sleep(1)
    Thread(target=get_color, args=(color_event, )).start()

    while not btn.is_active:
        sleep(0.1)

    while motors.turns < 3:
        moveUntilDist()
        turn()
        motors.get_current_turn(mpu.currentAngle)
        sleep(0.1)
        new_J = new_J + 1

    print("Finishing")
    starting_point_adjustment()
    sleep(10)
    dist_event.set()
    color_event.set()
    color_reco.stop = True

if __name__ == '__main__':

    green_center = (-1, -1, -1, -1, -1, -1)
    red_center = (-1, -1, -1, -1, -1, -1)

    new_J = 0

    frontDist = 0
    rightDist = 0
    leftDist = 0

    debugList = [False, False]

    motors = mctrl(motorPins=[24, 13], servoPin=12, speed=1, debug=debugList[0])
    mpu = MPU(gyro=250, acc=2, tau=0.98, debug=debugList[1])

    color_reco = ColorsCoordinations().start()
    btn = Button(20)

    main()
