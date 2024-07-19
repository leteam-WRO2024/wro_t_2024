from gpiozero import Motor, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from typing import Union, NewType
from time import sleep
from pid_ctrl import pidc


class mctrl():
    def __init__(self, motorPins: list, servoPin: int, speed: int = 1, debug: bool = False) -> None:
        self.motorPins: list = motorPins
        self.servoPin: int = servoPin

        # OLD pidc vals: 0.6, 0.0, 0.3 True
        self.motor: Motor = Motor(*self.motorPins)
        self.factory = PiGPIOFactory()
        self.servo = AngularServo(self.servoPin, min_angle=0, max_angle=180,min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=self.factory)

        # 0.8789 0.04 0.048
        # self.pid = pidc(0.6, 0.038, 0.034, True)
        self.pid = pidc(0.9, 0, 0.02, True)

        self.distance_threshold = 28
        self.debug = debug

        self.__speed = speed
        self.__direction = 0

        self.min_angle = 40
        self.max_angle = 150
        self.mid_angle = 100
        
        self.turns = 0

    @property
    def angle(self):
        return self.servo.angle

    @angle.setter
    def angle(self, val):
        self.servo.angle = self.map_angle(val)

    @property
    def direction(self):
        return self.__direction

    @direction.setter
    def direction(self, val):
        if val not in [1, 0, -1]:
            self.__direction = 0
            print("Direction should be in [1, 0, -1] ... Resseting to 0")
        self.__direction = val

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, val):
        self.__speed = round(min(1, max(0, val)), 2)

    def minfo(self, val):
        if self.debug:
            print(val)

    def get_next_angle(self, angle: Union[int, float], ninety_deg: bool = False):
        next_angle = ((angle + 10) + (self.direction * 90)) % 360
        if ninety_deg:
            next_angle = (round(next_angle / 90)) * 90

        return next_angle

    def get_heading_angle(self, angle: Union[float, int]):
        return (self.get_next_angle(angle, True) - (self.direction * 90)) % 360
    
    def get_current_turn(self, angle: Union[float, int]):
        if self.get_heading_angle(angle = angle) == 0:
            self.turns += 1
            
    def map_angle(self, angle: Union[int, float]):
        return max(self.min_angle, min(self.max_angle, angle))

    def adjust_angle(self, heading: Union[int, float], reading: Union[int, float], right_distance: Union[int, float], left_distance: Union[int, float]):

        distance_error = 0

        if left_distance < 82 and right_distance < 82:
            if self.direction == -1:
                distance_error = self.distance_threshold - left_distance
            elif self.direction == 1:
                distance_error = right_distance - self.distance_threshold
            else:            
                if 0 < right_distance < self.distance_threshold:
                    distance_error = right_distance - 25
                elif 0 < left_distance < self.distance_threshold:
                    distance_error = 25 - left_distance

        # if right_distance < 82 and left_distance < 82:
        #     if self.direction == -1:
        #         distance_error = self.distance_threshold - left_distance
        #     elif self.direction == 1:
        #         distance_error = right_distance - self.distance_threshold
        #     else:
        #         distance_error = right_distance - left_distance

        # distance_error = self.pid.calc_pid(distance_error)
        # print(distance_error)
        # Other ideas
        # add the distance error to the error_val before doing the -180 - 180 degrees mapping
        error_val = reading - heading + distance_error
        error_val = ((error_val + 180) % 360) - 180
        error_val = (error_val + self.mid_angle)
        # print(error_val, distance_error)

        # Other ideas
        # - error_val += 85 + distance_error
        # + self.angle = error_val + distance_error
        # needs the servo angle to be reset to 90 degrees continuously
        self.angle = error_val

    def direction_turn(self):
        if self.direction == -1:
            self.turn_right()
        elif self.direction == 1:
            self.turn_left()

    # Needs more work i think never tested it fully
    def fix_forward(self, heading: Union[float, int] = 0, reading: Union[float, int] = 0, left_distance: Union[float, int] = 0, right_distance: Union[float, int] = 0):
        print("\n\n::::::::Inside fix, forward::::::::\n\n")

        if left_distance > right_distance:
            self.turn_right()
        elif right_distance > left_distance:
            self.turn_left()

        self.move_backward(nspeed=1)
        sleep(1)
        self.reverse_angle()
        self.move_forward(nspeed=0.8)
        sleep(0.84)
        for _ in range(3):
            self.adjust_angle(heading, reading, left_distance, right_distance)
            sleep(0.005)

    def fix_stuck(self, heading: Union[float, int] = 0, reading: Union[float, int] = 0, left_distance: Union[float, int] = 0, right_distance: Union[float, int] = 0):
        self.stop_car()
        self.reverse_angle()
        self.move_backward(nspeed=1)
        sleep(1)
        self.reverse_angle()
        self.move_forward()

    def reverse_angle(self):
        self.angle = (self.mid_angle - self.angle) + self.mid_angle

    def turn_forward(self):
        self.angle = self.mid_angle

    def turn_left(self):
        self.angle = self.min_angle

    def turn_right(self):
        self.angle = self.max_angle

    def move_forward(self, nspeed: Union[int, None] = None):
        self.motor.forward(nspeed if nspeed else self.speed)

    def move_backward(self, nspeed: Union[int, None] = None):
        self.motor.backward(nspeed if nspeed else self.speed)

    def stop_car(self):
        self.motor.stop()
