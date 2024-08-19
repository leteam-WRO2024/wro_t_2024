from gpiozero import Motor, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from typing import Union, NewType
from time import sleep, time
from pid_ctrl import pidc
from global_vals import *
from numpy import exp


class mctrl():
    def __init__(self, motorPins: list, servoPin: int, speed: int = 1, debug: bool = False) -> None:
        self.motorPins: list = motorPins
        self.servoPin: int = servoPin
        self.debug = debug

        # OLD pidc vals: 0.6, 0.0, 0.3 True
        self.motor: Motor = Motor(*self.motorPins)
        self.factory = PiGPIOFactory()
        self.servo = AngularServo(self.servoPin, min_angle=0, max_angle=180,
                                  min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=self.factory)

        # 0.8789 0.04 0.048
        # self.pid = pidc(0.6, 0.038, 0.034, True)

        #-> old setpoint = 130
        self.color_pidc = pidc(0.21, 0, 0.005, 80, 70, False) 
        # self.color_pidc = pidc(0.1, 0, 0.1, 150, 40, True)
        
        self.turns = 0

        self.__speed = speed
        self.__direction = 0

        self.min_angle = 50
        self.max_angle = 140
        self.mid_angle = 110
        
        
        self.total_weight = 0

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

    def calculate_weighted_error(self, color, weight_sign):
        error = 0
        center_x_obj, _, _, _, w, h = color
        if center_x_obj == -1:
            return 0

        
        weight = w * h
        if weight_sign == 1:
            if center_x_obj > (FRAME_CENTER - 40):
                center_away = (FRAME_CENTER - 40) - center_x_obj
            else:
                center_away = center_x_obj - (FRAME_CENTER - 40)
        else:
            center_away = center_x_obj
            
        error = (center_away) * weight
        self.total_weight += weight
     
        # if self.total_weight != 0:
        #     error /= self.total_weight

        error /= (FRAME_WIDTH)
        print(f"Error before pidc: {error}", end = " ,")
        error = self.color_pidc.calc_pid(error) 
        print(f"angle = {error}, weight = {weight}, center_away = {center_away}")
        return error

    def color_based_adjustment(self, heading: Union[float, int], reading: Union[float, int], color, weight_sign, left_dist, right_dist):
        error = self.calculate_weighted_error(color, weight_sign)     

        if error == 0:
            if left_dist == 0:
                left_dist = 200
            if right_dist == 0:
                right_dist = 200

            left_dist = min(left_dist, 50)
            right_dist = min(right_dist, 50)

            error = 40 - left_dist

        angle_error = reading - heading
        # if left_dist < 15:
        #     angle_error += 20
        # elif right_dist < 15:
        #     angle_error -= 20
        
        angle_error = ((angle_error + 180) % 360) - 180
        angle_error = (angle_error + self.mid_angle) + error
        self.angle = angle_error
        # self.angle = self.smooth_transition(self.angle, angle_error)
        
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

if __name__ == "__main__":
    
    motors = mctrl(motorPins=[24, 13], servoPin=12, speed=1, debug=True)
    motors.angle = motors.mid_angle
