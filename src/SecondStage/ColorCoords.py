# import the necessary packages
import cv2
import numpy as np
from threading import Thread
from imutils.video import FPS
from queue import Queue
from time import sleep
from global_vals import *

# ColorsCoordinations(resolution=(320, 240), framerate=32,awb_mode="auto", brightness=60)
COLOR_RANGES = {
    "green":    (np.array([40, 40, 20], np.uint8), np.array([95, 255, 205], np.uint8)),
    "red":      (np.array([160, 110, 60], np.uint8), np.array([1757, 255, 255], np.uint8)),
    "orange":   (np.array([0, 30, 50], np.uint8), np.array([24, 255, 255], np.uint8)),
    "blue":     (np.array([100, 70, 51], np.uint8), np.array([130, 255, 255], np.uint8)),
    "black":    (np.array([0, 0, 0], np.uint8), np.array([180, 255, 50], np.uint8))
}


class ColorsCoordinations:
    def __init__(self):

        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.frame = None
        self.__stop = False

        self.x, self.y, self.coords, self.h, self.w = -1, -1, (-1, -1), -1, -1

    @property
    def stop(self):
        return self.__stop

    @stop.setter
    def stop(self, val: bool):
        self.__stop = val

    def __cropped_frame(self, iFrame, crop_width = CROP_WIDTH, crop_height = CROP_HEIGHT):
        height, width, _ = iFrame.shape
        right_crop = width - crop_width
        top_crop = iFrame[crop_height: height, :]
        return top_crop[:, crop_width: top_crop.shape[1] - crop_width]
    
    def __crop_right(self, iFrame, right_width: CROP_RIGHT): # type: ignore
        height, width, _ = iFrame.shape
        return iFrame[:, width - right_width:]
    
    def __crop_left(self, iFrame, left_width: CROP_LEFT): # type: ignore
        height, width, _ = iFrame.shape
        return iFrame[:, :left_width]

    def __crop_top(self, iFrame, crop_height = CROP_HEIGHT):
        height, width, _ = iFrame.shape
        return iFrame[crop_height: height, :]

    def __segment_frame(self, frame, lower: np.ndarray, upper: np.ndarray):
        kernel = np.ones((5, 5), "uint8")
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvFrame, lower, upper)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        return mask

    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        while True:
            isRead, frame = self.stream.read()
            self.frame = cv2.resize(frame, (400, 300))

            if self.stop:
                break
            
            sleep(0.000002)
            
    def get_centroid(self, contours):
        if len(contours) == 0:
            return (-1, -1, -1, -1, -1, -1)
        
        max_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(max_contour) <= 400:
            return (-1, -1, -1, -1, -1, -1)

        x, y, w, h = cv2.boundingRect(max_contour)

        center_x = x + w / 2
        center_y = y + h / 2

        return (center_x, center_y, x, y, w, h)

            # cent_queue.appendleft(center)

    def crop(self, frame, width, height):
        # rframe = self.__crop_top(frame)
        rframe = self.__cropped_frame(frame, width, height)
        return rframe
    
    def get_color_ranges(self, color: str):
        if color not in list(COLOR_RANGES.keys()):
            print(
                f"\nWARNING!! .. specified color ({color}) is not in the predefined ranges\n")

        return COLOR_RANGES.get(color, [0, 0, 0])
    
    def get_orange_blue(self, lower: np.ndarray, upper: np.ndarray):
        imageFrame = self.read()

        imageFrame = self.__cropped_frame(imageFrame)

        imageFrame = cv2.GaussianBlur(imageFrame, (1, 1), 0)
        mask = self.__segment_frame(imageFrame, lower, upper)
        cv2.imshow("test", imageFrame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for pic, contour in enumerate(contours):
            if (cv2.contourArea(contour) > 600):
                self.x, self.y, self.w, self.h = cv2.boundingRect(contour)

        self.coords = (self.x, self.y)
        self.x = self.y = self.w = self.h = -1

        return self.coords
    
    def get_red_green(self, lower: np.ndarray, upper: np.ndarray):
        imageFrame = self.read()

        # imageFrame = self.__crop_top(imageFrame)
        imageFrame = self.__cropped_frame(imageFrame, 85, 140)
        # cv2.flip()
        imageFrame = cv2.GaussianBlur(imageFrame, (1, 1), 0)
        mask = self.__segment_frame(imageFrame, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # x, y, w, h = cv2.boundingRect(max(contours, key = cv2.contourArea))
        # cv2.rectangle(imageFrame,(x,y),(x+w,y+h),(0,255,0),2)
        # cv2.putText(imageFrame,'Moth Detected',(x+w+10,y+h),0,0.3,(0,255,0))
        # cv2.imshow("test", imageFrame)
        
        
        return self.get_centroid(contours=contours)
    
    def get_black(self, lower: np.ndarray, upper: np.ndarray):
        imageFrame = cv2.GaussianBlur(self.read(), (1, 1), 0)

        mask = self.__segment_frame(imageFrame, lower, upper)

        left_region = self.__crop_left(mask)
        right_region = self.__crop_right(mask)

        left_contours  = cv2.findContours(left_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        right_contours = cv2.findContours(right_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # DO LOGIC FOR HANDELLING WHEN THERE IS NO WALL OR RETURN SOMETHING TO DO THE LOGIC ON ANITHER PLACE

    def read(self):
        return self.frame

    def detect_color(self, color: str):
        if color in ["red", "green"]:
            return self.get_red_green(*self.get_color_ranges(color))
        elif color in ["blue", "orange"]:
            return self.get_orange_blue(*self.get_color_ranges(color))
        elif color == "black":
            return self.get_black(*self.get_color_ranges(color))

if __name__ == "__main__":
    import os
    image = ColorsCoordinations().start()
    sleep(2)

    while True:
        frame = image.read()
        if frame is not None:
            x = image.detect_color("red")
            # print(x)
            cv2.imshow("test", image.crop(frame, 85, 140))
            # os.system("clear")
            
            
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
