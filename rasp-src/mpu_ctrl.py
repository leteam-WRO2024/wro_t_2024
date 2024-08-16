import smbus
import math
from time import sleep, time
import threading
from typing import Union


class MPU:

    def __init__(self, gyro: int, acc: int, tau: float, currentAngle: int = -1, debug: bool = False):
        self.__currentAngle = currentAngle
        self.__nextAngle = 0

        self.debug = debug

        self.gx = None
        self.gy = None
        self.gz = None
        self.ax = None
        self.ay = None
        self.az = None

        self.gyroXcal = 0
        self.gyroYcal = 0
        self.gyroZcal = 0

        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.dtTimer = 0
        self.tau = tau

        self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(gyro)
        self.accScaleFactor, self.accHex = self.accelerometerSensitivity(acc)

        self.bus = smbus.SMBus(1)
        self.address = 0x68

    @property
    def currentAngle(self):
        return self.__currentAngle

    @currentAngle.setter
    def currentAngle(self, val):
        val = (val + 360) % 360
        self.__currentAngle = int(val)

    @property
    def nextAngle(self):
        return self.__nextAngle

    @nextAngle.setter
    def nextAngle(self, val):
        self.__nextAngle = val

    def isReady(self):
        while self.currentAngle == -1:
            print("Awaiting Calibration")
            sleep(0.15)

        return True

    def dinfo(self, value):
        if self.debug:
            print(value)

    def get_next_angle(self, angle: Union[int, float], direction: int = 0,ninety_deg: bool = False):
        next_angle = ((angle + 10) + (direction * 90)) % 360
        if ninety_deg:
            next_angle = (round(next_angle / 90)) * 90

        return next_angle

    def gyroSensitivity(self, x):
        # Create dictionary with standard value of 500 deg/s
        return {
            250:  [131.0, 0x00],
            500:  [65.5,  0x08],
            1000: [32.8,  0x10],
            2000: [16.4,  0x18]
        }.get(x,  [65.5,  0x08])

    def accelerometerSensitivity(self, x):
        # Create dictionary with standard value of 4 g
        return {
            2:  [16384.0, 0x00],
            4:  [8192.0,  0x08],
            8:  [4096.0,  0x10],
            16: [2048.0,  0x18]
        }.get(x, [8192.0,  0x08])

    def setUp(self):
        # Activate the MPU-6050
        self.bus.write_byte_data(self.address, 0x6B, 0x00)

        # Configure the accelerometer
        self.bus.write_byte_data(self.address, 0x1C, self.accHex)

        # Configure the gyro
        self.bus.write_byte_data(self.address, 0x1B, self.gyroHex)

        # Display message to user
        print("MPU set up:")
        print('\tAccelerometer: ' + str(self.accHex) +
              ' ' + str(self.accScaleFactor))
        print('\tGyro: ' + str(self.gyroHex) +
              ' ' + str(self.gyroScaleFactor) + "\n")
        sleep(2)

    def eightBit2sixteenBit(self, reg):
        # Reads high and low 8 bit values and shifts them into 16 bit
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg+1)
        val = (h << 8) + l

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def getRawData(self):
        self.gx = self.eightBit2sixteenBit(0x43)
        self.gy = self.eightBit2sixteenBit(0x45)
        self.gz = self.eightBit2sixteenBit(0x47)

        self.ax = self.eightBit2sixteenBit(0x3B)
        self.ay = self.eightBit2sixteenBit(0x3D)
        self.az = self.eightBit2sixteenBit(0x3F)

    def calibrateGyro(self, points):
        # Display message
        print("Calibrating gyro with " + str(points) + " points. Do not move!")

        # Take N readings for each coordinate and add to itself
        for _ in range(points):
            self.getRawData()
            self.gyroXcal += self.gx
            self.gyroYcal += self.gy
            self.gyroZcal += self.gz

        # Find average offset value
        self.gyroXcal /= points
        self.gyroYcal /= points
        self.gyroZcal /= points

        # Display message and restart timer for comp filter
        print("Calibration complete")
        print("\tX axis offset: " + str(round(self.gyroXcal, 1)))
        print("\tY axis offset: " + str(round(self.gyroYcal, 1)))
        print("\tZ axis offset: " + str(round(self.gyroZcal, 1)) + "\n")
        sleep(2)
        self.dtTimer = time()
        self.currentAngle = 0

    def processIMUvalues(self):
        # Update the raw data
        self.getRawData()

        # Subtract the offset calibration values
        self.gx -= self.gyroXcal
        self.gy -= self.gyroYcal
        self.gz -= self.gyroZcal

        # Convert to instantaneous degrees per second
        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor

        # Convert to g force
        self.ax /= self.accScaleFactor
        self.ay /= self.accScaleFactor
        self.az /= self.accScaleFactor

    def compFilter(self, a):
        # Get the processed values from IMU
        self.processIMUvalues()

        # Get delta time and record time for next call
        dt = time() - self.dtTimer
        self.dtTimer = time()

        # Acceleration vector angle
        accPitch = math.degrees(math.atan2(self.ay, self.az))
        accRoll = math.degrees(math.atan2(self.ax, self.az))
        accYaw = math.degrees(math.atan2(self.ay, self.ax))

        # Gyro integration angle
        self.gyroRoll -= self.gy * dt
        self.gyroPitch += self.gx * dt
        self.gyroYaw += self.gz * dt
        self.yaw = self.gyroYaw

        # Comp filter
        self.roll = (self.tau) * (self.roll - self.gy * dt) + \
            (1-self.tau) * (accRoll)

        self.pitch = (self.tau) * (self.pitch + self.gx * dt) + \
            (1 - self.tau) * (accPitch)

        self.yaw = (self.tau) * (self.yaw + self.gz * dt) + \
            (1 - self.tau) * accYaw

        return self.yaw


def loop(mpu: MPU):
    mpu.setUp()
    mpu.calibrateGyro(500)

    while True:
        mpu.currentAngle = mpu.compFilter(1)


if __name__ == '__main__':
    mpu = MPU(gyro=250, acc=2, tau=0.98, debug=True)

    calThread = threading.Thread(target=loop, args=(mpu, )).start()

    if mpu.isReady():
        while True:
            mpu.nextAngle = mpu.get_next_angle(mpu.currentAngle, -1,True)
            print(mpu.currentAngle, mpu.nextAngle)
            sleep(1)

    print("Closing")
