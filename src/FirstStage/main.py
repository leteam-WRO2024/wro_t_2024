from ultrasonic_ctrl import ultra
from mpu_ctrl import MPU
from encoder import Encoder
from gpiozero import Button, LED
from motor_ctrl import mctrl
from time import sleep, time
from threading import Thread, Event
from ColorCoords import ColorsCoordinations
import cv2
import numpy as np

new_J = 0
COLOR_RANGES = {
    "green":    (np.array([55, 173, 35], np.uint8), np.array([90, 255, 205], np.uint8)),
    "red":      (np.array([0, 51, 64], np.uint8), np.array([180, 255, 255], np.uint8)),
    "orange":   (np.array([2, 40, 60], np.uint8), np.array([30, 255, 255], np.uint8)),
    "blue":     (np.array([90, 105, 90], np.uint8), np.array([125, 255, 255], np.uint8))
}

def read_ultra(event: Event):
    while not event.is_set():
        frontUltra.distance = frontUltra.calcDistance()
        leftUltra.distance = leftUltra.calcDistance()
        rightUltra.distance = rightUltra.calcDistance()

def mpu_loop(mpu: MPU):
    global new_J
    mpu.setUp()
    mpu.calibrateGyro(500)

    while True:
        try:
            mpu.currentAngle = mpu.compFilter(1) + ((3 * new_J) * motors.direction)
            # mpu.currentAngle = mpu.compFilter(1)
            sleep(0.000007)
        except:
            
            pass

def get_color(event: Event):
    motors.direction = 0
    while not event.is_set():
        orange = color_reco.detect_color("orange")
        blue = color_reco.detect_color("blue")
        if orange != (-1, -1):
            motors.direction = -1
        elif blue != (-1, -1):
            motors.direction = 1

        if motors.direction != 0:
            event.set()
            color_reco.stop = True
        # print(f"direction: {motors.direction}")


def starting_point_adjustment():
    heading = motors.get_heading_angle(mpu.currentAngle)
    motors.move_forward(0.7)


    if frontUltra == 0:
        sleep(1)
    elif frontUltra > 145:
        while (frontUltra > 145 or frontUltra == 0):
            pass
        # motors.adjust_angle(heading, mpu.currentAngle, rightUltra, leftUltra)
            # print("adjusting")
    print("out")

    motors.stop_car()
    motors.turn_forward()


def moveUntilDist():
    heading = motors.get_heading_angle(mpu.currentAngle)
    motors.adjust_angle(heading, mpu.currentAngle, rightUltra, leftUltra)

    motors.move_forward()

    motors.minfo(f"\n\nFRONT:: {frontUltra}\n\n")
    while True:
            # print(f"Moving with speed {motors.speed}  --- front {frontUltra}")
        if (frontUltra < 82 and frontUltra != 0) or (leftUltra > 120 or rightUltra > 120):
            break
        motors.adjust_angle(heading, mpu.currentAngle, rightUltra, leftUltra)
                
        motors.minfo(
            f"\nFRONT:: {frontUltra}\nRIGHT:: {rightUltra}\nLEFT:: {leftUltra}\n")

    print("OUT OF LOOP")
    motors.turn_forward()
    motors.stop_car()
    motors.speed = 1
    
def turn():

    heading = motors.get_heading_angle(mpu.currentAngle)

    mpu.nextAngle = motors.get_next_angle(heading, True)
    motors.direction_turn()
    sleep(0.003)
     
    motors.move_forward(1)

    mpu.dinfo(
        f"ABOVE:: ({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")
    
    while not (250 > (mpu.currentAngle - mpu.nextAngle) * motors.direction >= -5):
        mpu.dinfo(
            f"({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")
        sleep(0.00005)
        

    mpu.dinfo(
        f"UNDER:: ({mpu.currentAngle} - {mpu.nextAngle}) * {motors.direction} = {(mpu.nextAngle - mpu.currentAngle) * motors.direction}")

    motors.turn_forward()
    motors.stop_car()
    sleep(0.36)

def main():
    global new_J
    # start_sleep = 0.44

    motors.turn_forward()
    motors.speed = 1
    
    color_event = Event()
    dist_event = Event()

    Thread(target=mpu_loop, args=(mpu, )).start()

    ready = mpu.isReady()

    Thread(target=get_color, args=(color_event, )).start()
    Thread(target=read_ultra, args=(dist_event, )).start()

    # while not btn.is_active:
    #     sleep(0.1)

    while motors.turns < 3:
        moveUntilDist()
        # sleep(start_sleep)
        turn()
        motors.get_current_turn(mpu.currentAngle)
        sleep(0.1)
        # print(f"Turns: {motors.turns}")
        new_J = new_J + 1
    
    print("Finishing")
    starting_point_adjustment()
    sleep(1)
    dist_event.set()


if __name__ == '__main__':
    new_J = 0

    frontUltra = ultra(10, 26)
    rightUltra = ultra(22, 16)
    leftUltra  = ultra(9, 11)

    debugList = [False, False]

    motors = mctrl(motorPins=[24, 13], servoPin=12, speed=1, debug=debugList[0])
    mpu = MPU(gyro=250, acc=2, tau=0.98, debug=debugList[1])
    enc = Encoder(15)

    color_reco = ColorsCoordinations().start()
    btn = Button(20)

    main()
