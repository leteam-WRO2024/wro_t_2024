# Class to monitor a rotary encoder and update a value.  You can either read the value when you need it, by calling getValue(), or
# you can configure a callback which will be called whenever the value changes.

import RPi.GPIO as GPIO
from time import sleep, time


class Encoder:

    def __init__(self, rightPin, callback=None):
        self.rightPin = rightPin
        self.callback = callback

        self.__value = 0
        self.__displacement = 0
        self.state = 0
        self.__evalues: tuple = (0, 0)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.rightPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH,
                              callback=self.transitionOccurred)

    @property
    def evalues(self):
        return self.__evalues

    @evalues.setter
    def evalues(self, val):
        if not isinstance(val, tuple):
            raise TypeError("Value must be of type tuple")

        self.__evalues = val

    @property
    def value(self):
        return self.__value

    @value.setter
    def value(self, val):
        self.__value = val

    @property
    def displacement(self):
        return self.__displacement

    @displacement.setter
    def displacement(self, val):
        self.__displacement = val

    def transitionOccurred(self, channel):

        self.value += 1
        self.displacement = self.value // 23

        self.evalues = (self.value, self.displacement)


if __name__ == "__main__":
    enc = Encoder(21)
    while True:
        print(enc.evalues)
        sleep(1)
