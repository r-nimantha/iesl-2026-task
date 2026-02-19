import socket
import struct
import numpy as np
import cv2
import time
import threading

try:
    from pupil_apriltags import Detector
    APRILTAGS_AVAILABLE = True
except ImportError:
    APRILTAGS_AVAILABLE = False

class CameraStream:
    def __init__(self, host="localhost", port=5599, capture_count=10, interval=1.0):
        self.host = host
        self.port = port
        self.capture_count = capture_count
        self.interval = interval
        
        # Detection variables
        self.current_frame = None
        self.lock = threading.Lock()
        
        # Line detection results
        self.line_center_x = None
        self.line_center_y = None
        self.line_angle = None
        self.line_found = False
        
        # April tag detection results
        self.april_tag_found = False
        self.april_tag_id = None
        self.april_tag_center = None
        
        # Initialize April tag detector
        if APRILTAGS_AVAILABLE:
            self.detector = Detector(families='tag36h11')
        else:
            self.detector = None
            print("Warning: pupil-apriltags not installed. April tag detection disabled.")

    def detect_line(self, frame):
        try:
            #  Edge detection (critical change)
            blurred = cv2.GaussianBlur(frame, (9, 9), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Clean edges slightly
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            edges = cv2.dilate(edges, kernel, iterations=1)

            # Probabilistic Hough Transform
            lines = cv2.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi / 180,
                threshold=50,
                minLineLength=100,
                maxLineGap=20
            )

            if lines is None:
                with self.lock:
                    self.line_found = False
                return False

            # Choose longest detected line
            longest = max(lines, key=lambda l: np.hypot(
                l[0][2] - l[0][0],
                l[0][3] - l[0][1]
            ))

            x1, y1, x2, y2 = longest[0]

            # Compute center
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # Compute angle
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))

            # Normalize angle
            if angle < -90:
                angle += 180
            elif angle > 90:
                angle -= 180

            with self.lock:
                self.line_center_x = cx
                self.line_center_y = cy
                self.line_angle = angle
                self.line_found = True

            return True

        except Exception as e:
            print(f"Line detection error: {type(e).__name__}: {e}")

        with self.lock:
            self.line_found = False

        return False


    def detect_april_tags(self, frame):
        """Detect April tags in the frame."""
        if self.detector is None:
            return False

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = self.detector.detect(gray)

            if len(detections) > 0:
                det = detections[0]

                center = det.getCenter()
                tag_id = det.getId()

                print("AprilTag detected")
                print(f"Tag ID: {tag_id}")

                with self.lock:
                    self.april_tag_found = True
                    self.april_tag_id = tag_id
                    self.april_tag_center = center

                return True

        except Exception:
            pass

        with self.lock:
            self.april_tag_found = False

        return False


    def get_line_info(self):
        """Get current line detection info. Returns (found, center_x, center_y, angle)."""
        with self.lock:
            return self.line_found, self.line_center_x, self.line_center_y, self.line_angle

    def get_april_tag_info(self):
        """Get current April tag detection info. Returns (found, tag_id, center_x, center_y)."""
        with self.lock:
            if self.april_tag_found and self.april_tag_center:
                return self.april_tag_found, self.april_tag_id, self.april_tag_center[0], self.april_tag_center[1]
            return False, None, None, None

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("127.0.0.1", 5599))

        header_size = struct.calcsize("=HH")
        while True:
            header = s.recv(header_size)
            if len(header) != header_size:
                print("Header size mismatch")
                break

            width, height = struct.unpack("=HH", header)

            bytes_to_read = width * height
            img = bytes()
            while len(img) < bytes_to_read:
                img += s.recv(min(bytes_to_read - len(img), 4096))

            img = np.frombuffer(img, np.uint8).reshape((height, width))

            with self.lock:
                self.current_frame = img.copy()

            # Run detections
            self.detect_line(img)
            self.detect_april_tags(img)

            cv2.imshow("image", img)
            if cv2.waitKey(1) == ord("q"):
                break
        s.close()
