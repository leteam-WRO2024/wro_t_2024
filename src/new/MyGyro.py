##### --Import Libraries-- ######

import time
import serial
from adafruit_bno08x_rvc import BNO08x_RVC  # pylint:disable=wrong-import-position
from threading import Thread


##### --initialization-- ######
class BNO085:
    uart = serial.Serial("/dev/ttyAMA0", 115200)
    rvc = BNO08x_RVC(uart)
    uart.close()
    myyaw=0
    def start(self):
        Thread(target=self.Read_loop, args=()).start()
    def Read_Yaw(self):
        return self.myyaw
    def Read_loop(self):
        while True:
            self.Read_Yaw2()
            time.sleep(0.01)
    def Read_Yaw2(self):
        yaw="err"
        while yaw=="err":
            try:
                self.uart.open()
                yaw, _, _, _, _, _ = self.rvc.heading
                yaw=yaw+180
                #print("Yaw: %2.2f Degrees" % (yaw))
                #print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
                #print("")
                self.uart.close()

            except:
                #print("err")
                yaw="err"
                self.uart = serial.Serial("/dev/ttyAMA0", baudrate=115200)
                self.rvc = BNO08x_RVC(self.uart)

            self.uart.close()
            self.myyaw= yaw
