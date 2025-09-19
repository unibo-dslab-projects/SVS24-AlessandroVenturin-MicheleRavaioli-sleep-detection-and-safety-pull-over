from utils import EyeStateDetector, CameraStream
from threading import Thread, Lock
import numpy as np
import cv2

class InattentionDetector():
    """Detect if the driver is focused on the road or not.
    """
    def __init__(self, cam_stream: CameraStream, eye_threshold=0.15):
        self.cam = cam_stream
        self.detector = EyeStateDetector(eye_threshold)
        self.__is_inattent = False
        self.__is_busy = False
        self.__lock = Lock()

    def detect(self) -> bool:
        """Detect if the driver is focused on the road or not. The image must be grayscale.
           Returns is_inattent.
        """
        def do_detection():
            with self.__lock:
                self.__is_busy = True

            img_gray = self.cam.next_grayscale()
            results = self.detector.predict(img_gray)

            is_inattent = False
            for detection in results:
                if detection.label == 1: # eyes are open
                    is_inattent = True

            self.__is_inattent = is_inattent

            with self.__lock:
                self.__is_busy = False


        with self.__lock:
            if not self.__is_busy:
                Thread(target=do_detection).start()
        
        return self.__is_inattent

