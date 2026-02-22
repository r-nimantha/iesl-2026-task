import socket
import struct
import numpy as np
import cv2
import time
import threading
import apriltag

class CameraStream:
    def __init__(self, host="localhost", port=5599, capture_count=10, interval=1.0):
        self.host = host
        self.port = port
        self.capture_count = capture_count
        self.interval = interval
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
        self.detector = apriltag.Detector()

    def detect_line(self, frame):
        try:
            blurred = cv2.GaussianBlur(frame, (9, 9), 0)
            edges = cv2.Canny(blurred, 50, 150)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            edges = cv2.dilate(edges, kernel, iterations=1)
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

            rightmost = max(lines, key=lambda l: (l[0][0] + l[0][2])/2)
            x1, y1, x2, y2 = rightmost[0]
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))

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
            print(f"[LINE_DETECTION_ERROR] {type(e).__name__}: {e}", flush=True)

        with self.lock:
            self.line_found = False

        return False


    def detect_april_tags(self, frame):
        try:
            if self.detector is None:
                print("[APRILTAG_ERROR] Detector not initialized!", flush=True)
                return False
            
            detections = self.detector.detect(frame)
            if len(detections) == 0:
                with self.lock:
                    self.april_tag_found = False
                return False
            
            det = detections[0]
            tag_id = det.tag_id
            center = det.center
            corners = det.corners.astype(int)

            with self.lock:
                self.april_tag_found = True
                self.april_tag_id = tag_id
                self.april_tag_center = center

            return True

        except Exception as e:
            import traceback
            print(f"[APRILTAG_ERROR] {type(e).__name__}: {e}", flush=True)
            print(f"[APRILTAG_TRACEBACK] {traceback.format_exc()}", flush=True)

        with self.lock:
            self.april_tag_found = False

        return False


    def get_line_info(self):
        with self.lock:
            return self.line_found, self.line_center_x, self.line_center_y, self.line_angle

    def get_april_tag_info(self):
        with self.lock:
            if self.april_tag_found:
                return self.april_tag_found, self.april_tag_id, self.april_tag_center[0], self.april_tag_center[1]
            return False, None, None, None
    
    def reset_detections(self):
        with self.lock:
            self.line_found = False
            self.line_center_x = None
            self.line_center_y = None
            self.line_angle = None

            self.april_tag_found = False
            self.april_tag_id = None
            self.april_tag_center = None

    def run(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(("127.0.0.1", 5599))

            header_size = struct.calcsize("=HH")
            frame_count = 0
            while True:
                frame_count += 1
                header = s.recv(header_size)
                if len(header) != header_size:
                    print("[CAMERA] Header size mismatch, stopping", flush=True)
                    break

                width, height = struct.unpack("=HH", header)

                bytes_to_read = width * height
                img = bytes()
                while len(img) < bytes_to_read:
                    img += s.recv(min(bytes_to_read - len(img), 4096))

                img = np.frombuffer(img, np.uint8).reshape((height, width))

                self.detect_april_tags(img)
                self.detect_line(img)
                
                cv2.imshow("image", img)
                if cv2.waitKey(1) == ord("q"):
                    print("[CAMERA] 'q' pressed, stopping camera", flush=True)
                    break
            s.close()
            print("[CAMERA] Socket closed", flush=True)
        except ConnectionRefusedError as e:
            print(f"[CAMERA_ERROR] Connection refused - is the camera server running? {e}", flush=True)
        except Exception as e:
            print(f"[CAMERA_ERROR] {type(e).__name__}: {e}", flush=True)
