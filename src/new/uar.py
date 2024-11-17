import time

# uncomment and comment out the above for use with Raspberry Pi
import serial
uart = serial.Serial("/dev/serial0", 115200)


from adafruit_bno08x_rvc import BNO08x_RVC  # pylint:disable=wrong-import-position

rvc = BNO08x_RVC(uart)
uart.close()



while True:
    try:
        time.sleep(0.01)
        uart.open()
        print("t")

        yaw, _, _, _, _, _ = rvc.heading
        
        yaw=yaw+180
        print("Yaw: %2.2f Degrees" % (yaw))
        #print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
        print("")
        uart.close()

    except:
        print("err")
        uart = serial.Serial("/dev/serial0", baudrate=115200)
        rvc = BNO08x_RVC(uart)

        uart.close()

        time.sleep(0.01)
