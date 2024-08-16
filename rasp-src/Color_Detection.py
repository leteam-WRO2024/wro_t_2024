import cv2
import numpy as np
from threading import Thread, local
from time import sleep, time
from global_vals import *

class ColorDetectionThread(Thread):
    def __init__(self, color, frame_provider, result_dict, stop_flags):
        super().__init__()
        self.color = color
        self.frame_provider = frame_provider
        self.result_dict = result_dict
        self.local_storage = local()
        self.stop_flags = stop_flags

    def run(self):
        while not self.stop_flags.get(self.color, False):
            frame = self.frame_provider.read()
            if frame is None:
                continue

            lower, upper = COLOR_RANGES.get(self.color)
            imageFrame = self.__crop_frame(frame, *CROP_PARAMS.get(self.color, (0, 0, 0)))
            imageFrame_gaussed = cv2.GaussianBlur(imageFrame, (2, 2), cv2.BORDER_DEFAULT)
            mask = self.__segment_frame(imageFrame_gaussed, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            # Filter contours and store the result
            self.local_storage.result = {
                    "coords": self.filter_contours(contours),
                    "timestamp": time()}

    def __crop_frame(self, frame, crop_height=0, crop_right=0, crop_left=0):
        height, _, _ = frame.shape
        return frame[crop_height:height, crop_left:crop_right]
    
    def __segment_frame(self, frame, lower: np.ndarray, upper: np.ndarray):
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvFrame, lower, upper)
        kernel = np.ones((4, 4), "uint8")
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
        return mask

    def filter_contours(self, contours, min_area: int = 350):
        max_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(max_contour) <= min_area:
            return (-1, -1, -1, -1, -1, -1)

        x, y, w, h = cv2.boundingRect(max_contour)
        center_x = x + w / 2
        center_y = y + h / 2

        return (center_x, center_y, x, y, w, h)

class ColorsCoordinations:
    def __init__(self):
        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.frame = None
        self.__stop = False

    @property
    def stop(self):
        return self.__stop

    @stop.setter
    def stop(self, val: bool):
        self.__stop = val

    def update(self):
        while True:
            isRead, frame = self.stream.read()
            self.frame = cv2.resize(frame, (400, 300))
            if self.stop:
                break

    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def read(self):
        return self.frame

if __name__ == "__main__":
    colors = ["green", "red", "blue", "orange"]  # Add any other colors you need
    color_detection_threads = []
    results = {}
    stop_flags = {color: False for color in colors}  # Initialize stop flags

    # Create and start color detection threads
    image = ColorsCoordinations().start()
    sleep(2)  # Give it some time to start up

    for color in colors:
        thread = ColorDetectionThread(color, image.read, results, stop_flags)
        thread.daemon = True
        thread.start()
        color_detection_threads.append(thread)

    while True:
        frame = image.read()
        if frame is not None:
            cv2.imshow("Frame", frame)

            # Display detection results

            for thread in color_detection_threads:
                if hasattr(thread.local_storage, "result"):
                    results[thread.color] = thread.local_storage.result

            for color, coords in results.items():
                print(f"Detected {color} at: {coords}")

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    # Stop the color detection threads
    image.stop = True
    for thread in color_detection_threads:
        thread.join()

    cv2.destroyAllWindows()