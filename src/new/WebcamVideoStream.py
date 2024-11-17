from threading import Thread
import cv2
import imutils
import time
import numpy as np

class WebcamVideoStream:
	def __init__(self, src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
		imgsz = (1920, 1080)


		self.stream = cv2.VideoCapture(src)
		if type(imgsz) is tuple:
			self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, imgsz[0])
			self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, imgsz[1])
		self.stream.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
		self.stream.set(cv2.CAP_PROP_FPS, 120)

		(self.grabbed, self.frame) = self.stream.read()
		self.frame2=self.frame
		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self
	def update(self):
		# keep looping infinitely until the thread is stopped
		pframe=0
		nframe=0
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return
			# otherwise, read the next frame from the stream
			
			(self.grabbed, self.frame) = self.stream.read()
			self.frame2 = imutils.resize(self.frame, width=800)
			#self.frame2 = self.frame
			#cv2.GaussianBlur(self.frame, (5, 5), 0)
			#show fps
			'''
			nframe=time.time()
			fps=1/(nframe-pframe)
			pframe=nframe
			fps=int(fps)
			fps=str(fps)
			cv2.putText(self.frame2,fps,(7,25),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)
			'''

            
			
	def read(self):
		# return the frame most recently read
		return self.frame2
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
'''
b=WebcamVideoStream()
b.start()
while True:
    cv2.imshow("IN Frame", b.frame2)
    key = cv2.waitKey(1)
    if key ==27:
        break'''
