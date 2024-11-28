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
CW='none'

smallest_x = 0

def stop_script():
    print('Exiting')
    done.set()

def filter_func(contour):
    x, _, _, _ = cv2.boundingRect(contour)
    
    if x < smallest_x:
        return False
    
    return True

#parameters
min_angle=30
mid_angle=90
max_angle=130
class ColorsCoordinations:

    # allow the camera to warmup
    x=y=w=h=-1
    r_l=[54,98,35]
    r_u=[86,255,110]
    b_l=[0,0,0]
    b_u=[255,255,30]
    bu_l=[110,85,100]
    bu_u=[125,255,255]
    g_l=[170,50,73]
    g_u=[180,255,255]
    o_l=[0,100,110]
    o_u=[50,255,255]
    pu=[168,223,250]
    pl=[163,180,94]
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
    xR=-1
    xG=-1
    RorG=-1
    
    red_area = -1
    green_area = -1
    
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
                    #cv2.drawContours(imageFrame, c, -1, (0, 255, 0), 2)
                    self.x, self.y, self.w, self.h = cv2.boundingRect(c)
                else:
                    return -1,-1,-1,-1
        
            x=self.x 
            y=self.y
            w=self.w
            h=self.h
            self.x=self.y=-1
            if show:
                cv2.imshow(name,imageFrame)
                key = cv2.waitKey(delay) & 0xFF
            return x,y,w,h
    
    def getcoor_o(self,imageFrame,l,u,name,delay,show,minsize):
        while True:

            hsv_frame2 = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
            kernal = np.ones((5, 5), "uint8") 
            dilated_frame = cv2.dilate(hsv_frame2, kernal)
            dilated_frame = cv2.erode(dilated_frame, kernal)

            lower = np.array([l[0], l[1], l[2]], np.uint8) 
            upper = np.array([u[0], u[1], u[2]], np.uint8) 
            mask = cv2.inRange(dilated_frame, lower, upper) 

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            filterd = list(filter(filter_func, contours))
            if len(filterd)>0:
                
                c = max(filterd, key = cv2.contourArea)
                area = cv2.contourArea(c)
                if(area > minsize):
                    #if name == "f_green":
                    cv2.drawContours(imageFrame, c, -1, (255, 255, 255), 2)
                    self.x, self.y, self.w, self.h = cv2.boundingRect(c)
                else:
                    return -1,-1,-1,-1

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
    
    def obstacle_reverse(self,delay,show,heading):
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
        
        frame3=frame[90:450,250+diff:550+diff]

        self.xR,y1,wr,hr=c.getcoor_o(frame3,c.r_l,c.r_u,"f_red",delay,show,500)
        self.xG,y2,wg,hg=c.getcoor_o(frame3,c.g_l,c.g_u,"f_green",delay,show,500)
        #print(f"this is green{self.xG} this is red {self.xR}")
        self.xG += (wg // 2)
        if  self.xR==-1 and self.xG==-1:
            self.RorG='none'
            self.green_area = -1
            self.red_area = -1
        else:
            if (wr*hr) > (wg*hg):
                self.RorG='red'
                self.red_area = wr * hr
            else:
                self.RorG='green'
                #print(f"area: {wg*hg}")
                self.green_area = wg * hg
    
    def obstacle(self,delay,show,heading):
        #pframe=time.time()
        frame=self.cam.read()


        
        frame3=frame[90:450,0:800]

        self.xR,y1,wr,hr=c.getcoor_o(frame3,c.r_l,c.r_u,"f_red",delay,show,500)
        self.xG,y2,wg,hg=c.getcoor_o(frame3,c.g_l,c.g_u,"f_green",delay,show,500)
        #print(f"this is green{self.xG} this is red {self.xR}")
        self.xG += (wg // 2)
        if  self.xR==-1 and self.xG==-1:
            self.RorG='none'
            self.green_area = -1
            self.red_area = -1
        else:
            if (wr*hr) > (wg*hg):
                self.RorG='red'
                self.red_area = wr * hr
                self.green_area = -1
            else:
                self.RorG='green'
                #print(f"area: {wg*hg}")
                self.green_area = wg * hg
                self.red_area = -1
        #print(f"this is RorG {self.RorG}")
    def ground(self,delay,show,heading):
        global CW
        #pframe=time.time()
        frame=self.cam.read()


        #print(frame.shape)
        frame3=frame[370:400,380:420]
        frame3 = cv2.resize(frame3, (800, 600), interpolation = cv2.INTER_LINEAR)
        self.xb,y1,w,h=c.getcoor(frame3,c.bu_l,c.bu_u,"f_blue",delay,show,50)
        self.xo,y2,w,h=c.getcoor(frame3,c.o_l,c.o_u,"f_orange",delay,show,50)
        if CW=='none':
            if y2>y1:
                CW=True
            elif y1>y2:
                CW=False


c=ColorsCoordinations()
frame=c.cam.read()

time.sleep(0.1)
pframe=0
nframe=0
show=False
delay=1
side_values=[]

def turn90_halfback(side_angle, next_angle):
    
    speed = 200
    roter_limmit = 850
    stop()
    c.obstacle(delay,show,side_angle)
    rotor.steps=0
    
    
    while rotor.steps < 500:
        forward(speed)
        adjust_gyro(side_angle)
        
    if (c.xR > -1 and 200 < c.xR) or (c.xG > -1 and 200 < c.xG):
        while c.xR > -1 or c.xG > -1:
            #print("t", c.xR, c.xG, sep = ",")
            adjust_gyro(side_angle)
            forward(255)
            c.obstacle(delay,show,side_angle)
        
        
        rotor.steps = 0
        while rotor.steps < roter_limmit:
            #print("t", c.xR, c.xG, sep = ",")
            adjust_gyro(side_angle)
            forward(255)
            c.obstacle(delay,show,side_angle)
        
        stop()
        
        while rotor.steps > -1000:
            backward(speed)
            adjust_gyro_backward(next_angle)
            
    else:
        rotor.steps = 0
        stop()
        go_left(speed)
        
        while rotor.steps < roter_limmit:
            forward(speed)
            
        go_right(speed)

        while  rotor.steps > 0:
            #print("Here", rotor.steps, sep = ": ")
            backward(speed)
            adjust_gyro_backward(next_angle)    

        stop()
    
    c.obstacle(delay,show,side_angle)

    
    if c.xG != -1 or c.xR != -1:
        go_right(speed)

        #while  (c.green_area > 2700 or c.red_area > 2700) and (c.green_area < 4000 or c.red_area < 4000):
        while (2800 < c.green_area < 4000) or (2800 < c.red_area < 4000): 
            print(f"green_area: {c.green_area}")
            print(f"red_area: {c.red_area}")
            backward(speed)
            adjust_gyro_backward(next_angle)    

            c.obstacle(delay,show,side_angle)
            
        stop()

def turn90_halfbackc(side_angle, next_angle):

    speed = 200
    roter_limmit = 850
    stop()
    c.obstacle(delay,show,side_angle)
    rotor.steps=0
    
    while rotor.steps < 500:
        forward(speed)
        adjust_gyro(side_angle)    

    if (c.xR > -1 and c.xR < 700) or (c.xG > -1 and c.xG < 700):
        while c.xR > -1 or c.xG > -1:
            print("t", c.xR, c.xG, sep = ",")
            adjust_gyro(side_angle)
            forward(255)
            c.obstacle(delay,show,side_angle)
        
        
        rotor.steps = 0
        while rotor.steps < roter_limmit:
            print("t", c.xR, c.xG, sep = ",")
            adjust_gyro(side_angle)
            forward(255)
        
        while rotor.steps > -1000:
            adjust_gyro_backward(next_angle)
            backward(speed)
            
    else:
        rotor.steps=0
        stop()
        go_right(speed)
        
        while rotor.steps < roter_limmit:
            forward(speed)
            
        go_left(speed)

        while rotor.steps > 0:
            backward(speed)
            adjust_gyro_backward(next_angle)    


        stop()

    c.obstacle(delay, show ,side_angle)
        
    if c.xG != -1 or c.xR != -1:
        go_left(speed)

        #while  (c.green_area > 2700 or c.red_area > 2700) and (c.green_area < 4000 or c.red_area < 4000):
        while (2800 < c.green_area < 4000) or (2800 < c.red_area < 4000): 
            print(f"green_area: {c.green_area}")
            print(f"red_area: {c.red_area}")
            backward(speed)
            adjust_gyro_backward(next_angle)    

            c.obstacle(delay,show,side_angle)
            
        stop()
            

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
    #forward(speed)
def go_left(speed):
    print("am going left")
    set_servo_angle(min_angle)
    #forward(speed)
def go_right_r(speed):
    set_servo_angle(min_angle)
    backward(speed)
def go_left_r(speed):
    set_servo_angle(max_angle)
    backward(speed)
def lastrun(side_angle):
    c.front(delay,show,side_angle)

    rotor.steps=0
    color_Err = 0
    yf=c.yf
    code = f"a:{mid_angle}$"
    ser.write(code.encode("utf-8"))
    speed=255
    while rotor.steps < 3850:
        
        speed_offset=0#int((rotor.steps/900)*80)
        speed=speed-speed_offset
        
        if speed<180:
            speed=180
        
        code = f"f:{speed}$"    
        ser.write(code.encode("utf-8"))

        c.obstacle(delay,show,side_angle)
        color_Err = 0
        
        if (c.xG != -1) and (c.xG > 160):
            color_Err = (c.xG) // 4
        elif c.xR != -1 and c.xR < 540:
            print("i am adjusting")
            color_Err = (c.xR - 800) // 2

        err=0
        Gyro_angle=c.imu.Read_Yaw()
 
        if not Gyro_angle=="err":
            err=(side_angle-Gyro_angle)
            if err>100:
                err=err-360
            elif err<-100:
                err=err+360
                
        dist_err = 0
        if c.xl > 70 and c.xl != -1:
            dist_err = -(70 - c.xl) * 15
        elif c.xr < 250 and c.xr != -1:
            #print("am here")
            dist_err = -(250 - c.xr) * 6
        
  
        servo_angle = mid_angle + err + color_Err + dist_err

        if servo_angle>max_angle:
            servo_angle=max_angle
        if servo_angle<min_angle:
            servo_angle=min_angle

        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))
        
    code = f"s:{speed}$"    
    ser.write(code.encode("utf-8"))
    print(rotor.steps)

def adjust_gyro(side_angle):
        Gyro_angle=c.imu.Read_Yaw()
        err = 0
        if not Gyro_angle=="err":
            err=(side_angle-Gyro_angle)
        else:
            return
        if err > 100:
            err = err - 360
        elif err < -100:
            err = err + 360
  
        servo_angle=mid_angle+err

        if servo_angle>max_angle:
            servo_angle=max_angle
        if servo_angle<min_angle:
            servo_angle=min_angle

        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))

def adjust_gyro_backward(side_angle):
        Gyro_angle=c.imu.Read_Yaw()
        err = 0
        if not Gyro_angle=="err":
            err=(side_angle-Gyro_angle)
        else:
            return
        if err>100:
            err=err-360
        elif err<-100:
            err=err+360
  
        servo_angle=mid_angle - err

        if servo_angle>max_angle:
            servo_angle=max_angle
        if servo_angle<min_angle:
            servo_angle=min_angle

        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))


def onesidedrive_c(side_angle):
    global first_Side
    global CW
    #c.front(delay,show,side_angle)
    roter_limit=2400
    if first_Side:
        first_Side=False
        roter_limit=0
    c.boarders(delay,show)
    rotor.steps=0
    #yf=c.yf
    code = f"a:{mid_angle}$"
    ser.write(code.encode("utf-8"))
    speed=255
    ptime=time.time()
    c.ground(delay,show,side_angle)
    
    while  True:
        color_Err=0
        if( not c.xo==-1 or not c.xb==-1) and rotor.steps>roter_limit:
            rotor.steps=0
            
            if  (c.xr != -1 ):
                while rotor.steps<800:
                    adjust_gyro(side_angle)
                    forward(255)
                    #print("im here1")
                break
            else:
                break
            
        c.ground(delay,show,side_angle)
        c.obstacle(delay,show,side_angle)
        
        if (c.xG != -1) and (c.xG > 160):
            color_Err = (c.xG) // 4
        elif c.xR != -1 and c.xR < 540:
            print("i am adjusting")
            color_Err = (c.xR - 800) // 2
        
        print(c.RorG)
        
        speed_offset=0#int((rotor.steps/900)*80)
        speed=speed-speed_offset
        if speed<180:
            speed=180
               
        code = f"f:{speed}$"    
        ser.write(code.encode("utf-8"))
        err=0
        Gyro_angle=c.imu.Read_Yaw()
        dist_err = 0

        
        if not Gyro_angle=="err":
            err=(side_angle-Gyro_angle)
            if err>100:
                err=err-360
            elif err<-100:
                err=err+360
        
        if c.xl > 70 and c.xl != -1:
            dist_err = -(70 - c.xl) * 15
        elif c.xr < 250 and c.xr != -1:
            #print("am here")
            dist_err = -(250 - c.xr) * 6
        
        print(f"dist_err: {dist_err}")
  
        servo_angle = mid_angle + err + dist_err + color_Err

        if servo_angle>max_angle:
            servo_angle=max_angle
        if servo_angle<min_angle:
            servo_angle=min_angle

        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))
        c.boarders(delay,show)
        #c.front(delay,show,side_angle)
        yf=c.yf
        ftime=time.time()
        fps=1/(ftime-ptime)
        ptime=ftime
        #print(f"fps: {fps} {c.xr}{c.xl}")
        
    code = f"s:{speed}$"    
    ser.write(code.encode("utf-8"))
    print(rotor.steps)
    
def onesidedrive(side_angle):
    global first_Side
    global CW
    #c.front(delay,show,side_angle)
    roter_limit=4000
    if first_Side:
        first_Side=False
        roter_limit=0
    c.boarders(delay,show)
    rotor.steps=0
    #yf=c.yf
    code = f"a:{mid_angle}$"
    ser.write(code.encode("utf-8"))
    
    speed=255
    ptime=time.time()
    c.ground(delay,show,side_angle)
    
    while  True:
        color_Err = 0
        dist_err = 0
        if( not c.xo==-1 or not c.xb==-1) and rotor.steps>roter_limit:
            rotor.steps=0
        
            
            if  (c.xl!=-1 ):
                while rotor.steps<780:
                    adjust_gyro(side_angle)
                    forward(255)
                    #print("im here1")
                break
            else:
                break
            '''
            else:
                while rotor.steps<500:
                    adjust_gyro(side_angle)
                    forward(255)
                    #print("im here2")
                    continue
                break
            '''
               
        c.ground(delay,show,side_angle)
        c.obstacle(delay,show,side_angle)
        
        if c.RorG == "green" and (not c.xG==-1) and (c.xG > 160):
            color_Err=(c.xG) // 3
        elif c.xR != -1 and c.xR < 540:
            #print("i am adjusting")
            color_Err=(c.xR-800) // 2
        print(c.RorG)
        
        if c.xl > 70 and c.xl != -1:
            dist_err = -(70 - c.xl) * 15
        elif c.xr < 200 and c.xr != -1:
            #print("am here")
            dist_err = -(200 - c.xr) * 2
    
            
        speed_offset=0#int((rotor.steps/900)*80)
        speed=speed-speed_offset
        if speed<180:
            speed=180
        code = f"f:{speed}$"    
        ser.write(code.encode("utf-8"))
        
        err=0
        Gyro_angle=c.imu.Read_Yaw()
        
        
        if not Gyro_angle=="err":
            err = (side_angle - Gyro_angle)
            if err > 100:
                err = err - 360
            elif err < -100:
                err = err + 360
                
        print(f"dist_err: {dist_err}")
        servo_angle = mid_angle + err + dist_err + color_Err

        if servo_angle>max_angle:
            servo_angle=max_angle
        if servo_angle<min_angle:
            servo_angle=min_angle

        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))
        c.boarders(delay,show)
        #c.front(delay,show,side_angle)
        yf=c.yf
        ftime=time.time()
        fps=1/(ftime-ptime)
        ptime=ftime
        #print(f"fps: {fps} {c.xr}{c.xl}")
        
    code = f"s:{speed}$"    
    ser.write(code.encode("utf-8"))
    print(f"one side {rotor.steps}")
    
def ccw_run():

    for i in range(3):
        onesidedrive(side_values[0])
        turn90_halfback(side_values[0], side_values[3])
        onesidedrive(side_values[3])
        turn90_halfback(side_values[3], side_values[2])
        onesidedrive(side_values[2])
        turn90_halfback(side_values[2], side_values[1])
        onesidedrive(side_values[1])
        turn90_halfback(side_values[1], side_values[0])
    lastrun(side_values[0])
    print(time.time()-t1)
    
def cw_run():

    for i in range(3):
        onesidedrive_c(side_values[0])
        turn90_halfbackc(side_values[0], side_values[1])
        onesidedrive_c(side_values[1])
        turn90_halfbackc(side_values[1], side_values[2])
        onesidedrive_c(side_values[2])
        turn90_halfbackc(side_values[2], side_values[3])
        onesidedrive_c(side_values[3])
        turn90_halfbackc(side_values[3], side_values[0])
    lastrun(side_values[0])
    print(time.time() - t1)
    
def firstsidedrive(side_angle):
    global first_Side
    global CW
    c.front(delay,show,side_angle)
    roter_limit=2400
    if first_Side:
        first_Side=False
        roter_limit=1000
    c.boarders(delay,show)
    c.ground(delay,show,side_angle)
    rotor.steps=0
    yf=c.yf
    code = f"a:{mid_angle}$"
    ser.write(code.encode("utf-8"))
    speed=255
    ptime=time.time()
    print(CW)
    while CW=='none':
        print(CW)
        dist_err = 0
        c.ground(delay,show,side_angle)
        
        speed_offset=0#int((rotor.steps/900)*80)
        speed=speed-speed_offset
        if speed<180:
            speed=180
        
        if c.xl > 70 and c.xl != -1:
            dist_err = -(70 - c.xl) * 10
        elif c.xr < 200 and c.xr!=-1:
            print("am here")
            dist_err = -(200 - c.xr) * 2
        
        
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
        servo_angle=mid_angle + err + dist_err
        code = f"a:{servo_angle}$"
        ser.write(code.encode("utf-8"))
        c.boarders(delay,show)
        #c.front(delay,show,side_angle)
        yf=c.yf
        ftime=time.time()
        fps=1/(ftime-ptime)
        ptime=ftime
        #print(f"fps: {fps} {c.xr}{c.xl}")
        '''
    if c.xo==-1:
        print("CCW")
        CW=False
    else:
        print("CW")
        CW=True
'''
    code = f"s:{speed}$"    
    ser.write(code.encode("utf-8"))
    print(rotor.steps)
    
while True:
    if  gpo.input(btn):
        time.sleep(1)
        t1=time.time()
        heading=c.imu.Read_Yaw()
        while heading=='err':
           heading=c.imu.Read_Yaw()
        calculate_sides_values(heading)

        '''
        while True:
            c.obstacle(delay,True,heading)
            print(c.xG)
            time.sleep(0.5)
        '''

        firstsidedrive(heading)

        first_Side=True

        if CW:
            cw_run()
        else:
            print("am in ccw")
            ccw_run()
    break
