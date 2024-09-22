from ultrasonic_ctrl import ultra
from mpu_ctrl import MPU
from gpiozero import Button, LED
from motor_ctrl import mctrl
from time import sleep, time
from threading import Thread, Event
from ColorCoords import ColorsCoordinations
import cv2
import numpy as np
from encoder import Encoder

new_J = 0


def read_ultra():
    while True:
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
            sleep(0.00005)
        except:
            pass


def get_color(event: Event):
    global green_center, red_center, turn_bool
    motors.direction = 0

    while True:
        orange = color_reco.detect_color("orange")
        blue = color_reco.detect_color("blue")
        if not event.is_set():
            if orange != (-1, -1):
                motors.direction = -1
            elif blue != (-1, -1):
                motors.direction = 1
            # else:
            #     if leftDist - rightDist > 83:
            #         motors.direction = 1
            #     elif rightDist - leftDist > 83:
            #         motors.direction = -1

        if motors.direction != 0 and not event.is_set():
            event.set()
        
        if orange != (-1, -1):
            turn_bool = True
        elif blue != (-1, -1):
            turn_bool = True


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
    global turn_bool
    heading = motors.get_heading_angle(mpu.currentAngle)

    motors.move_forward()

    motors.minfo(f"\n\nFRONT:: {frontDist}\n\n")
    while True:
        if green_center[2] != -1:
            motors.color_based_adjustment(heading, mpu.currentAngle, green_center, 1, leftDist, rightDist)
        elif red_center[2] != -1:
            motors.color_based_adjustment(heading, mpu.currentAngle, red_center, 1, leftDist, rightDist)
            motors.minfo(
                f"\nFRONT:: {frontDist}\nRIGHT:: {rightDist}\nLEFT:: {leftDist}\n")
        else:
            if turn_bool and frontDist < 70:
                if new_J == 0:
                    break
                else:
                    if (enc.displacement - last_displace) > 45:
                        break
                    else:
                        motors.color_based_adjustment(heading, mpu.currentAngle, (-1, -1, -1, -1, -1, -1), -1, leftDist, rightDist)
            else:
                motors.color_based_adjustment(heading, mpu.currentAngle, (-1, -1, -1, -1, -1, -1), -1, leftDist, rightDist)
            
            
    motors.turn_forward()
    motors.stop_car()
    motors.speed = 0.75
    print("OUT OF LOOP")


def turn():
    global last_displace, turn_bool
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
    last_displace = enc.displacement
    turn_bool = False
    motors.turn_forward()
    motors.stop_car()
    sleep(0.5)


def main():
    global new_J

    motors.max_angle = 150
    motors.min_angle = 50
    motors.mid_angle = 110
    motors.speed = 0.44
    motors.turn_forward()

    color_event = Event()
    dist_event = Event()

    Thread(target=mpu_loop, args=(mpu, )).start()
 
    ready = mpu.isReady()

    Thread(target=read_distance, args=(dist_event, )).start()
    sleep(1)
    Thread(target=get_color, args=(color_event, )).start()

    # while not btn.is_active:
    #     sleep(0.1)
    
    while motors.turns < 3:
        moveUntilDist()
        sleep(0.4)
        turn()
        motors.get_current_turn(mpu.currentAngle)
        new_J = new_J + 1

    # while True:
    #     motors.color_based_adjustment(0, 0, red_center, -1, leftDist, rightDist)

    print("Finishing")
    starting_point_adjustment()
    sleep(10)
    dist_event.set()
    color_event.set()
    color_reco.stop = True

if __name__ == '__main__':

    green_center = (-1, -1, -1, -1, -1, -1)
    red_center = (-1, -1, -1, -1, -1, -1)
    orange = (-1, -1)
    blue   = (-1, -1)
     
    new_J = 0

    frontUltra = ultra(10, 26)
    rightUltra = ultra(22, 16)
    leftUltra  = ultra(9, 11)


    turn_bool = False

    debugList = [False, False]

    motors = mctrl(motorPins=[24, 13], servoPin=12, speed=1, debug=debugList[0])
    mpu = MPU(gyro=250, acc=2, tau=0.98, debug=debugList[1])

    enc = Encoder(15)
    last_displace = 0
    
    color_reco = ColorsCoordinations().start()
    btn = Button(20)

    main()
