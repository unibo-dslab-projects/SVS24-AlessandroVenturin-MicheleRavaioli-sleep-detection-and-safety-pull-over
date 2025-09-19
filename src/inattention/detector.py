from .utils import EyeStateDetector, CameraStream
from threading import Lock, Thread
from typing import Optional
import numpy as np
import cv2


class InattentionDetector:
    """
    Detect whether the driver is inattentive (not focused on the road),
    based on eye state predictions from frames captured by a CameraStream.
    """

    def __init__(self, cam_stream: "CameraStream", eye_threshold: float = 0.15):
        """
        Parameters
        ----------
        cam_stream : CameraStream
            Source of images (grayscale frames are used for detection).
        eye_threshold : float
            Threshold parameter passed to EyeStateDetector (e.g., for
            determining if eyes are open or closed).
        """
        self.cam = cam_stream
        self.detector = EyeStateDetector(eye_threshold)

        # Internal state flags
        self._is_inattent: bool = False   # Latest detection result
        self._is_busy: bool = False       # True if a detection thread is currently running
        self._lock = Lock()               # Lock for synchronizing thread-safe state updates

    def detect(self) -> bool:
        """
        Run an asynchronous detection if not already in progress.

        Returns
        -------
        bool
            The most recent detection result (True if inattentive, False if attentive).

        Notes
        -----
        - This method spawns a background thread for detection so that calls
          do not block waiting for processing.
        - If called while detection is ongoing, it will simply return the last
          available result.
        """

        def do_detection():
            """Worker thread that performs the actual detection."""
            with self._lock:
                self._is_busy = True

            try:
                img_gray = self.cam.next_grayscale()
                results = self.detector.predict(img_gray)

                # Default: driver is not attentive
                is_inattent = True
                for detection in results:
                    if detection.label == 1:  
                        is_inattent = False

                self._is_inattent = is_inattent

            finally:
                with self._lock:
                    self._is_busy = False

        # Only start a new detection if not already busy
        with self._lock:
            if not self._is_busy:
                Thread(target=do_detection, daemon=True).start()

        # Return the last known result (may be slightly stale if detection is running)
        return self._is_inattent



class WebcamCameraStream(CameraStream):
    """
    Webcam camera stream using OpenCV.

    Parameters
    ----------
    device : int
        Camera device index (default 0).
    width, height : Optional[int]
        If provided, attempt to set capture resolution.
    flip : bool
        If True, flip the frame horizontally (useful for webcams).
    """

    def __init__(
        self,
        device: int = 0,
        width: Optional[int] = None,
        height: Optional[int] = None,
        flip: bool = False,
    ):
        self.device = device
        self.flip = flip

        self._cap = cv2.VideoCapture(self.device)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera device {self.device}")

        # Optionally set resolution
        if width is not None:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height is not None:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))


    def next(self) -> np.ndarray:
        """
        Return next color frame (BGR) as a numpy ndarray.

        Raises RuntimeError if frame could not be read.
        """
        ret, frame = self._cap.read()
        if not ret or frame is None:
            raise RuntimeError("Failed to read frame from camera.")
        if self.flip:
            frame = cv2.flip(frame, 1)
        return frame

    def next_grayscale(self) -> np.ndarray:
        """
        Return the next frame converted to grayscale.
        """
        color = self.next()
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        return gray

    def close(self) -> None:
        """Release camera."""
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    # Context manager support
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
