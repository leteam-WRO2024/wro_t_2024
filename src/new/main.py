import imutils
import time
import cv2
import numpy as np
from WebcamVideoStream import WebcamVideoStream as ws
from threading import Thread
from MyGyro import BNO085 as BN
from gpiozero import RotaryEncoder
from threading import Event
import RPi.GPIO as gpo
gpo.setmode(gpo.BCM)
from time import sleep
btn=6
gpo.setup(btn,gpo.IN)
import serial
#initialize arduino communications
ser = serial.Serial('/dev/ttyACM0', 2000000)
ser.flush()
first_Side=True
ser.reset_input_buffer()
time.sleep(2)
if not ser.is_open:
    ser.open()
    time.sleep(2)
#initialize Encoder
rotor = RotaryEncoder(19, 13, wrap=True, max_steps=18000)
rotor.steps = 0
done = Event()
CW=-1

def stop_script():
    print('Exiting')
    done.set()
    
#parameters
min_angle=30
mid_angle=90
max_angle=130
class ColorsCoordinations:

    # allow the camera to warmup
    x=y=w=h=-1
    g_l=[54,98,35]
    g_u=[86,255,255]
    b_l=[0,0,0]
    b_u=[255,255,30]
    bu_l=[110,85,100]
    bu_u=[125,255,255]
    r_l=[170,50,73]
    r_u=[180,255,255]
    o_l=[0,100,110]
    o_u=[50,255,255]
    #Ignore the warning message
    cam=ws()
    cam.start()
    xr=-1
    yr=-1
    xl=-1
    yl=-1
    yf=-1
    xo=-1
    xb=-1
    imu=BN()
    imu.start()
    def getcoor(self,imageFrame,l,u,name,delay,show,minsize):
        while True:

            hsv_frame2 = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
            kernal = np.ones((5, 5), "uint8") 
            dilated_frame = cv2.dilate(hsv_frame2, kernal)
            dilated_frame = cv2.erode(dilated_frame, kernal)

            lower = np.array([l[0], l[1], l[2]], np.uint8) 
            upper = np.array([u[0], u[1], u[2]], np.uint8) 
            mask = cv2.inRange(dilated_frame, lower, upper) 

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            if len(contours)>0:
                c = max(contours, key = cv2.contourArea)
                area = cv2.contourArea(c)
                if(area > minsize):
                    cv2.drawContours(imageFrame, c, -1, (0, 255, 0), 2)
                    self.x, self.y, self.w, self.h = cv2.boundingRect(c)

        
            x=self.x 
            y=self.y
            w=self.w
            h=self.h
            self.x=self.y=-1
            if show:
                cv2.imshow(name,imageFrame)
                key = cv2.waitKey(delay) & 0xFF
            return x,y,w,h
    def boarders(self,delay,show):
        frame=self.cam.read()
        #print(frame.shape)
        #frame1=frame
        frame1=frame[420:450,0:400]
        frame2=frame[420:450,400:800]
        #show fps
        '''
        nframe=time.time()
        fps=1/(nframe-pframe)
        pframe=nframe
        fps=int(fps)
        fps=str(fps)
        cv2.putText(frame2,fps,(7,25),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1,cv2.LINE_AA)
        '''
        xl, self.yl, lw, lh=c.getcoor(frame1,c.b_l,c.b_u,"f1",delay,show,300)
        
        if xl != -1:
            self.xl = (xl + lw) // 2
        else:
            self.xl = -1
        
        self.xr, self.yr, rw, rh=c.getcoor(frame2,c.b_l,c.b_u,"f2",delay,show,300)
        
    def front(self,delay,show,heading):
        frame=self.cam.read()

        curr=self.imu.Read_Yaw()
        while curr=='err':
            curr=self.imu.Read_Yaw()
        diff=heading-curr
        #print(diff)
        if diff>300:
            diff=360-diff
        elif diff<-300:
            diff=diff+360
            
        #print(diff)
        diff=int(diff)*4
        if diff>300:
            diff=300
        elif diff<-300:
            diff=-300
        
        frame3=frame[90:450,350+diff:450+diff]

        x3,y3,w,h=c.getcoor(frame3,c.b_l,c.b_u,"f3",delay,show,1000)
        #print(y3)
        self.yf=y3
    def ground(self,delay,show,heading):
        #pframe=time.time()
        frame=self.cam.read()


        
        frame3=frame[150:225,380:420]

        self.xb,y1,w,h=c.getcoor(frame3,c.bu_l,c.bu_u,"f_blue",delay,show,50)
        self.xo,y2,w,h=c.getcoor(frame3,c.o_l,c.o_u,"f_orange",delay,show,50)        
        


c=ColorsCoordinations()
frame=c.cam.read()

time.sleep(0.1)
pframe=0
nframe=0
show=False
delay=5
side_values=[]
def turn90_halfback():
    rotor.steps=0
    speed=200
    stop()
    roter_limmit=280
    go_left(speed)
    #while rotor.steps<roter_limmit:
        #ser.write(code.encode("utf-8"))
        #time.sleep(0.1)
    while  rotor.steps< roter_limmit:
        forward(speed)
    stop()

    go_right(speed)
    time.sleep(0.2)
    while  rotor.steps> roter_limmit:
        backward(speed)
    stop()
    time.sleep(0.2)
def turn90():
    rotor.steps=0
    speed=200
    stop()
    roter_limmit=470
    go_left(speed)
    #while rotor.steps<roter_limmit:
        #ser.write(code.encode("utf-8"))
        #time.sleep(0.1)
    while  rotor.steps< roter_limmit:
        forward(speed)
    stop()
def turn90c():
    rotor.steps=0
    speed=200
    stop()
    roter_limmit=470
    go_right(speed)
    #while rotor.steps<roter_limmit:
        #ser.write(code.encode("utf-8"))
        #time.sleep(0.1)
    while  rotor.steps< roter_limmit:
        forward(speed)
    stop()

def calculate_sides_values(first_side):
    side_values.append(first_side)
    for i in range(3):
        next_side_angle=side_values[i]+90
        if next_side_angle>360:
            next_side_angle=next_side_angle-360
        side_values.append(next_side_angle)
    print(side_values)
#communications with arduino test code
def forward(speed):
    code = f"f:{speed}$"    
    ser.write(code.encode("utf-8"))
def backward(speed):
    code = f"b:{speed}$"    
    ser.write(code.encode("utf-8"))
def stop():
    code = f"s: $"    
    ser.write(code.encode("utf-8"))
def set_servo_angle(angle):
    code = f"a:{angle}$"
    ser.write(code.encode("utf-8"))
def go_right(speed):
    set_servo_angle(max_angle)
    forward(speed)
def go_left(speed):
    print("am going left")
    set_servo_angle(min_angle)
    forward(speed)
def go_right_r(speed):
    set_servo_angle(min_angle)
    backward(speed)
def go_left_r(speed):
    set_servo_angle(max_angle)
    backward(speed)
def lastrun(side_angle):

    rotor.steps=0
    code = f"a:{mid_angle}$"
    ser.write(code.encode("utf-8"))
    speed=255
    while rotor.steps<500:
        speed_offset=0#int((rotor.steps/900)*80)
        speed=speed-speed_offset
        if speed<180:
            speed=180
        code = f"f:{speed}$"    
        ser.write(code.encode("utf-8"))
        err=0
        Gyro_angle=c.imu.Read_Yaw()
        if not Gyro_angle=="err":
            err=(side_angle-Gyro_angle)
            if err>100:
                err=360-err
            elif err<-100:
                err=err+360
        servo_angle=mid_angle+err
        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))
        c.boarders(delay,show)
        
    code = f"s:{speed}$"    
    ser.write(code.encode("utf-8"))
    print(rotor.steps)
    
def onesidedrive(side_angle):
    global first_Side
    global CW
    c.front(delay,show,side_angle)
    roter_limit=1200
    if first_Side:
        first_Side=False
        roter_limit=50
    c.boarders(delay,show)
    rotor.steps=0
    yf=c.yf
    code = f"a:{mid_angle}$"
    ser.write(code.encode("utf-8"))
    speed=255
    ptime=time.time()
    while yf==-1 or rotor.steps<roter_limit:
        speed_offset=0#int((rotor.steps/900)*80)
        speed=speed-speed_offset
        if speed<180:
            speed=180
        code = f"f:{speed}$"    
        ser.write(code.encode("utf-8"))
        err = 0
        Gyro_angle = c.imu.Read_Yaw()
        
        dist_err = 0
        
        if c.xl > 70 and c.xl != -1:
            dist_err = -(70 - c.xl) * 15
        elif c.xr < 200 and c.xr != -1:
            #print("am here")
            dist_err = -(200 - c.xr) * 4
            
        if not Gyro_angle=="err":
            err=(side_angle-Gyro_angle)
            if err>100:
                err=360-err
            elif err<-100:
                err=err+360

        servo_angle = mid_angle + err + dist_err

        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))
        c.boarders(delay,show)
        c.front(delay,show,side_angle)
        yf=c.yf
        ftime=time.time()
        fps=1/(ftime-ptime)
        ptime=ftime
        #print(f"fps: {fps} {c.xr}{c.xl}")
        
    code = f"s:{speed}$"    
    ser.write(code.encode("utf-8"))
    print(rotor.steps)
def ccw_run():

    for i in range(3):
        onesidedrive(side_values[0])
        turn90()
        onesidedrive(side_values[3])
        turn90()
        onesidedrive(side_values[2])
        turn90()
        onesidedrive(side_values[1])
        turn90()
    lastrun(side_values[0])
    print(time.time()-t1)
    
def cw_run():

    for i in range(3):
        onesidedrive(side_values[0])
        turn90c()
        onesidedrive(side_values[1])
        turn90c()
        onesidedrive(side_values[2])
        turn90c()
        onesidedrive(side_values[3])
        turn90c()
    lastrun(side_values[0])
    print(time.time()-t1)
    
def firstsidedrive(side_angle):
    global first_Side
    global CW
    c.front(delay,show,side_angle)
    roter_limit=1200
    if first_Side:
        first_Side=False
        roter_limit=500
    c.boarders(delay,show)
    c.ground(delay,show,side_angle)
    rotor.steps=0
    yf=c.yf
    code = f"a:{mid_angle}$"
    ser.write(code.encode("utf-8"))
    speed=100
    ptime=time.time()
    while c.xo==-1 and c.xb==-1:
        c.ground(delay,show,side_angle)
        speed_offset=0#int((rotor.steps/900)*80)
        speed=speed-speed_offset
        if speed<180:
            speed=180
        code = f"f:{speed}$"    
        ser.write(code.encode("utf-8"))
        err=0
        Gyro_angle=c.imu.Read_Yaw()
 
        if not Gyro_angle=="err":
            err=(side_angle-Gyro_angle)
            if err>100:
                err=360-err
            elif err<-100:
                err=err+360
        print(f"err {err}")
        servo_angle=mid_angle+err
        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))
        c.boarders(delay,show)
        c.front(delay,show,side_angle)
        yf=c.yf
        ftime=time.time()
        fps=1/(ftime-ptime)
        ptime=ftime
        #print(f"fps: {fps} {c.xr}{c.xl}")
    if c.xo==-1:
        print("CCW")
        CW=False
    else:
        print("CW")
        CW=True

    code = f"s:{speed}$"    
    ser.write(code.encode("utf-8"))
    print(rotor.steps)


while True:
    if not gpo.input(btn):
        time.sleep(1)
        t1=time.time()
        heading=c.imu.Read_Yaw()
        while heading=='err':
           heading=c.imu.Read_Yaw()
        calculate_sides_values(heading)



        firstsidedrive(heading)

        first_Side=True

        if CW:
            cw_run()
        else:
            print("am in ccw")
            ccw_run()
