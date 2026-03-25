from itertools import count
import socket
import struct
import numpy as np
import cv2
import apriltag
from config import IMG_WIDTH, IMG_HEIGHT

class CameraStream:
    def __init__(self, host="127.0.0.1", port=5599):
        self.host = host
        self.port = port
        self.height = IMG_HEIGHT
        self.width = IMG_WIDTH
        self.frame_shape = (IMG_HEIGHT, IMG_WIDTH)
        self.socket = None

        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.side_pairs = [(2, 3), (3, 0), (0, 1), (1, 2)]
        self.white_mask = None
        self.frame = None
        self.last_path_center_x = None
        self.frame_debug = None

    def run(self):
        self.connect()
        while True:
            if self.frame_debug is not None:
                cv2.imshow("Path Debug", self.frame_debug)
            frame = self.get_frame()
            # cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

    def disconnect(self):
        if self.socket:
            self.socket.close()
            self.socket = None

    def get_frame(self):
        header_size = struct.calcsize("=HH")
        header = self.socket.recv(header_size)
        if len(header) != header_size:
            return None

        width, height = struct.unpack("=HH", header)

        bytes_to_read = width * height
        img = bytes()
        while len(img) < bytes_to_read:
            img += self.socket.recv(min(bytes_to_read - len(img), 4096))
        
        self.frame = np.frombuffer(img, np.uint8).reshape((height, width))
        self.white_mask = (self.frame > 160).astype(np.uint8) * 255
        self.frame_debug = cv2.cvtColor(
            np.frombuffer(img, np.uint8).reshape((height, width)),
            cv2.COLOR_GRAY2BGR
        )
        return self.frame
    

    def compute_steering(self):
        cols_all = []
        rows_all = []

        window_half = self.width // 3
        if self.last_path_center_x is None:
            center_x = self.width // 2
        else:
            center_x = int(self.last_path_center_x)

        x_start = max(0, center_x - window_half)
        x_end   = min(self.width, center_x + window_half)

        strips = [
            (0, self.height // 10),
            (self.height * 2 // 10,     self.height * 3 // 10),
            (self.height * 4 // 10,     self.height // 2),
        ]

        for r_start, r_end in strips:
            roi = self.white_mask[r_start:r_end, x_start:x_end]
            cols = np.where(roi > 0)[1]
            if len(cols) > 0:
                cols_all.append(np.mean(cols) + x_start)
                rows_all.append((r_start + r_end) / 2)

        if len(cols_all) < 2:
            return None, None

        # Update last known path center using closest strip
        self.last_path_center_x = cols_all[0]

        coeffs = np.polyfit(rows_all, cols_all, deg=2)

        lookahead_row = self.height // 3
        lookahead_x = np.polyval(coeffs, lookahead_row)

        lateral_error = np.clip((lookahead_x - self.width / 2) / (self.width / 2), -1, 1)

        tangent = np.polyval(np.polyder(coeffs), lookahead_row)
        path_angle = np.arctan(tangent)
        yaw_rate = np.clip(lateral_error * 0.2 + path_angle * 0.7, -1, 1)

        return lateral_error, yaw_rate, path_angle * 180 / np.pi

    def detect_tag(self):
        results = self.detector.detect(self.frame)
        if results:
            return results[0]
        return None

    def get_tag_error(self):
        tag = self.detect_tag()
        if tag is None:
            return None

        tag_center = tag.center
        error_x = tag_center[0] / (self.width / 2) - 1
        error_y = tag_center[1] / (self.height / 2) - 1

        return error_x, error_y


    def compute_steering_debug(self):
        cols_all = []
        rows_all = []

        window_half = self.width // 5
        if self.last_path_center_x is None:
            center_x = self.width // 2
        else:
            center_x = int(self.last_path_center_x)

        x_start = max(0, center_x - window_half)
        x_end   = min(self.width, center_x + window_half)

        strips = [
            (0, self.height // 10),
            (self.height * 2 // 10, self.height * 3 // 10),
            (self.height * 4 // 10, self.height * 5 // 10),
            #(self.height * 6 // 10, self.height * 7 // 10),
        ]

        for r_start, r_end in strips:
            roi = self.white_mask[r_start:r_end, x_start:x_end]
            cols = np.where(roi > 0)[1]

            if len(cols) > 0:
                cols_all.append(np.mean(cols) + x_start)
                rows_all.append((r_start + r_end) / 2)

        if len(cols_all) < 2:
            return None, None, None

        self.last_path_center_x = cols_all[0]
        coeffs = np.polyfit(rows_all, cols_all, deg=2)
        lookahead_row = self.height // 2
        lookahead_x = np.polyval(coeffs, lookahead_row)
        lateral_error = np.clip(
            (lookahead_x - self.width / 2) / (self.width / 2),
            -1, 1
        )

        tangent = np.polyval(np.polyder(coeffs), lookahead_row)
        path_angle = np.arctan(tangent)

        yaw_rate = np.clip(
            lateral_error * 0.2 + path_angle * 0.7,
            -1, 1
        )

        # Draw ROI window (debug)
        cv2.rectangle(self.frame_debug, (x_start, 0), (x_end, self.height), (100, 100, 100), 1)

        # Draw sampled points (RED)
        for x, y in zip(cols_all, rows_all):
            cv2.circle(self.frame_debug, (int(x), int(y)), 6, (0, 0, 255), -1)

        # Draw polynomial curve (BLUE)
        y_vals = np.linspace(0, self.height - 1, 100).astype(int)
        x_vals = np.polyval(coeffs, y_vals)

        for i in range(len(y_vals) - 1):
            x1, y1 = int(x_vals[i]), int(y_vals[i])
            x2, y2 = int(x_vals[i+1]), int(y_vals[i+1])

            if 0 <= x1 < self.width and 0 <= x2 < self.width:
                cv2.line(self.frame_debug, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # Draw lookahead point (GREEN)
        cv2.circle(self.frame_debug, (int(lookahead_x), int(lookahead_row)), 8, (0, 255, 0), -1)

        # Draw heading direction (YELLOW arrow)
        arrow_length = 50
        dx = int(arrow_length * np.cos(path_angle))
        dy = int(arrow_length * np.sin(path_angle))

        cv2.arrowedLine(
            self.frame_debug,
            (int(lookahead_x), int(lookahead_row)),
            (int(lookahead_x + dx), int(lookahead_row + dy)),
            (0, 255, 255),
            2
        )

        # Draw center line (reference)
        cv2.line(
            self.frame_debug,
            (self.width // 2, 0),
            (self.width // 2, self.height),
            (0, 255, 255),
            1
        )
        return lateral_error, yaw_rate, path_angle * 180 / np.pi