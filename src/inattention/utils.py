import numpy as np
import cv2
import dlib
from pathlib import Path

class DetectionResult():
    """Result of the EyeDetector. Contains the landmark's coordinates of both eyes.
    """
    def __init__(self, left_eye, right_eye):
        self.left_eye = left_eye
        self.right_eye = right_eye

    def __repr__(self):
        return f'({self.left_eye}, {self.right_eye})'

class EyeDetector():
    """The detector of eye landmarks. Given a grayscale image, computes the 2D points for both eyes of all faces present.
    """
    def __init__(self):
        # face detector, necessary for landmark detection
        self.face_detector = dlib.get_frontal_face_detector()
        # 68 facial landmark detector
        path = Path(__file__).parent.absolute()
        self.shape_predictor = dlib.shape_predictor(str(path.joinpath('model/shape_predictor_68_face_landmarks.dat')))
        # these arrays contain the indeces of the keypoints we need
        self.left_eye_points = [36, 37, 38, 39, 40, 41]
        self.right_eye_points = [42, 43, 44, 45, 46, 47]

    def detect(self, gray_img):
        """Detect the landmarks for all faces. The image must be grayscale.
        """
        # detect faces in the grayscale image
        rects = self.face_detector(gray_img, 1)

        result = []
        # loop over the face detections
        for rect in rects:
            # determine the facial landmarks for the face region
            shape = self.shape_predictor(gray_img, rect)

            # extract the left and right eye coordinates (x, y)
            left_eye = [(shape.part(p).x, shape.part(p).y) for p in self.left_eye_points]
            right_eye = [(shape.part(p).x, shape.part(p).y) for p in self.right_eye_points]

            result.append(DetectionResult(left_eye, right_eye))

        return result
  

class ClassificationResult():
    """Result of the EyeClassifier. Contains the label (0=closed or 1=open) and the computed EAR.
    """
    def __init__(self, label, ear):
        self.label = label
        self.ear = ear

    def __repr__(self):
        return f'({self.label}, {self.ear})'

class EyeClassifier():
    """The eye classifier. Given a DetectionResult, classifies the eye between open and closed.
    """
    def __init__(self, threshold=0.1):
        # threshold not zero to account for errors
        self.threshold = threshold

    def distance(self, pointA, pointB):
	    # euclidean distance = norm2
        Ax, Ay = pointA
        Bx, By = pointB
        return np.sqrt((Bx - Ax)**2 + (By - Ay)**2)

    def eye_aspect_ratio(self, eye):
        """Compute the eye aspect ratio from the given set of landmarks. The input must be a list of six 2D points.
        """
        # distances between the two sets of vertical eye landmarks
        h1 = self.distance(eye[1], eye[5])
        h2 = self.distance(eye[2], eye[4])
        # distance between the horizontal eye landmark
        w = self.distance(eye[0], eye[3])
        # eye aspect ratio
        ear = (h1 + h2) / (2.0 * w)
        return ear
    
    def predict(self, detection):
        """Classify all the eyes detected as closed (0) or open (1). input must be an array of DetectionResult.
        """
        result = []
        for d in detection:
            # compute the mean eye aspect ratio
            left_ear = self.eye_aspect_ratio(d.left_eye)
            right_ear = self.eye_aspect_ratio(d.right_eye)
            ear = (left_ear + right_ear) / 2.0
            # classify the eye
            if ear < self.threshold:
                result.append(ClassificationResult(label=0, ear=ear))
            else:
                result.append(ClassificationResult(label=1, ear=ear))
        return result
  
class EyeStateDetectionResult():
    """Result of the Eye State Detector. Combines detection and classification results.
    """
    def __init__(self, detection_result: DetectionResult, classification_result: ClassificationResult):
        self.label = classification_result.label
        self.ear = classification_result.ear
        self.left_eye = detection_result.left_eye
        self.right_eye = detection_result.right_eye

class EyeStateDetector():
    """Detect the eye position and whether they are closed or open.
    """
    def __init__(self, threshold=0.1):
        self.detector = EyeDetector()
        self.classifier = EyeClassifier(threshold)

    def predict(self, gray_img):
        """Detect the eyes position and state in an input image. The image must be grayscale.
        """
        # execute detection and then classification
        detection_result = self.detector.detect(gray_img)
        classification_result = self.classifier.predict(detection_result)

        return [EyeStateDetectionResult(d, c) for (d, c) in zip(detection_result, classification_result)]
    
### CAMERA STREAM ###

class CameraStream():
    """This class represents a camera component, which manages streams of images."""

    def next(self) -> np.ndarray:
        """Return the next image of the stream as a numpy array."""
        raise NotImplementedError()

    def next_grayscale(self) -> np.ndarray:
        """Return the next image of the stream, in grayscale, as a numpy array."""
        raise NotImplementedError()
    
    def close(self) -> None:
        """Close the camera stream."""