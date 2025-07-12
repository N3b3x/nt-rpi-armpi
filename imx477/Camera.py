#!/usr/bin/env python3
from picamera2 import Picamera2
import cv2
import time
import numpy as np

class Camera:
    def __init__(self, resolution=(640, 480), picam2=None):
        self.resolution = resolution
        if picam2:
            self.picam2 = picam2
        else:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(main={"size": resolution})
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(1)  # give time to warm up

    def get_frame(self):
        frame = self.picam2.capture_array()
        return frame

if __name__ == '__main__':
    camera = Camera((640, 480))
    while True:
        frame = camera.get_frame()
        if frame is not None:
            cv2.imshow("IMX477 Camera", frame)
            if cv2.waitKey(1) == 27:  # ESC key
                break
    cv2.destroyAllWindows()