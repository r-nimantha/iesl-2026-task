from itertools import count
import socket
import struct
import numpy as np
from sympy import deg
import cv2
import apriltag
from config import IMG_WIDTH, IMG_HEIGHT

SIDE_NAMES = ["top", "right", "bottom", "left"]

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
            # if self.frame_debug is not None:
            #     cv2.imshow("Path Debug", self.frame_debug)
            frame = self.get_frame()
            cv2.imshow("Camera", frame)
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
        
        self.frame = np.frombuffer(img, np.uint8).reshape((height, width)).copy()
        blurred = cv2.GaussianBlur(self.frame, (5, 5), 0)
        self.white_mask = (blurred > 165).astype(np.uint8) * 255
        self.frame_debug = cv2.cvtColor(self.frame, cv2.COLOR_GRAY2BGR)
        return self.frame
        

    def compute_steering(self):
        cols_all = []
        rows_all = []

        window_half = self.width // 4
        center_x = int(self.last_path_center_x) if self.last_path_center_x is not None else self.width // 2
        x_start = max(0, center_x - window_half)
        x_end   = min(self.width, center_x + window_half)

        strips = [
            (0,                          self.height // 10),
            (self.height * 2 // 10,      self.height * 3 // 10),
            (self.height * 4 // 10,      self.height * 5 // 10),
        ]

        for r_start, r_end in strips:
            roi = self.white_mask[r_start:r_end, x_start:x_end]
            cols = np.where(roi > 0)[1]
            if len(cols) > 0:
                cols_all.append(np.mean(cols) + x_start)
                rows_all.append((r_start + r_end) / 2.0)

        if len(cols_all) < 2:
            return None, None, None

        self.last_path_center_x = cols_all[-1]

        t = np.arange(len(cols_all), dtype=float)
        deg = 2 if len(cols_all) >= 3 else 1
        coeffs_x = np.polyfit(t, cols_all, deg=deg)
        coeffs_y = np.polyfit(t, rows_all, deg=deg)

        t_lookahead = 0
        lookahead_x = np.polyval(coeffs_x, t_lookahead)
        lookahead_y = np.polyval(coeffs_y, t_lookahead)

        dx_dt = np.polyval(np.polyder(coeffs_x), t_lookahead) * -1
        dy_dt = np.polyval(np.polyder(coeffs_y), t_lookahead) * -1

        path_angle = np.arctan2(dx_dt, -dy_dt)

        lateral_error = np.clip(
            (lookahead_x - self.width / 2) / (self.width / 2),
            -1, 1
        )

        path_angle_normalized = path_angle / (np.pi / 2)
        yaw_rate = np.clip(
            lateral_error * 0.2 + path_angle_normalized * 0.7,
            -1, 1
        )

        return lateral_error, yaw_rate, np.degrees(path_angle)

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
    

    def get_path_angles(self):
        if self.white_mask is None:
            return []
        tag = self.detect_tag()
        if tag is None:
            return []
        center_x, center_y = tag.center
        tag_radius = np.hypot(tag.corners[0][0] - tag.corners[2][0], tag.corners[0][1] - tag.corners[2][1]) / 2
        sides_midpoints = [
            ((tag.corners[i][0] + tag.corners[j][0]) / 2, (tag.corners[i][1] + tag.corners[j][1]) / 2)
            for i, j in self.side_pairs
        ]
        radius = tag_radius * 1.5
        ring_mask = np.zeros_like(self.white_mask)
        cv2.circle(ring_mask, (int(center_x), int(center_y)), int(radius), 255, thickness=5)
        intersections = cv2.bitwise_and(self.white_mask, ring_mask)
        contours, _ = cv2.findContours(intersections, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        paths = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                raw_angle = np.degrees(np.arctan2(cx - center_x, -(cy - center_y)))
                distances = [
                    np.hypot(cx - mx, cy - my)
                    for mx, my in sides_midpoints
                ]
                closest_side = int(np.argmin(distances))
                side_name = SIDE_NAMES[closest_side]
                paths.append((raw_angle, side_name))
        # return only the first three elements, sorted by angle
        return sorted(paths, key=lambda x: x[0])[:3]


    def compute_steering_debug(self):
        cols_all = []
        rows_all = []

        window_half = self.width // 4
        center_x = int(self.last_path_center_x) if self.last_path_center_x is not None else self.width // 2
        x_start = max(0, center_x - window_half)
        x_end   = min(self.width, center_x + window_half)

        strips = [
            (0,                          self.height // 10),
            (self.height * 2 // 10,      self.height * 3 // 10),
            (self.height * 4 // 10,      self.height * 5 // 10),
        ]

        for r_start, r_end in strips:
            roi = self.white_mask[r_start:r_end, x_start:x_end]
            cols = np.where(roi > 0)[1]
            if len(cols) > 0:
                cols_all.append(np.mean(cols) + x_start)
                rows_all.append((r_start + r_end) / 2.0)

        if len(cols_all) < 2:
            return None, None, None

        # Update tracking with closest strip (last element)
        self.last_path_center_x = cols_all[-1]

        # Parametric fitting: both x and y as functions of t
        # t=0 is closest strip, t=last is furthest strip
        t = np.arange(len(cols_all), dtype=float)
        deg = 2 if len(cols_all) >= 3 else 1
        coeffs_x = np.polyfit(t, cols_all, deg=deg)  # x = f(t)
        coeffs_y = np.polyfit(t, rows_all, deg=deg)  # y = f(t)

        # Lookahead at furthest point (t = last index)
        t_lookahead = 0 #float(len(cols_all) - 1)
        lookahead_x = np.polyval(coeffs_x, t_lookahead)
        lookahead_y = np.polyval(coeffs_y, t_lookahead)

        # Tangent direction at lookahead
        dx_dt = np.polyval(np.polyder(coeffs_x), t_lookahead) * -1
        dy_dt = np.polyval(np.polyder(coeffs_y), t_lookahead) * -1

        # Angle from vertical (negative dy because y increases downward in image)
        path_angle = np.arctan2(dx_dt, -dy_dt)  # radians, 0 = straight ahead
        # if path_angle is not None: print(f"dx_dt: {dx_dt}\ndy_dt: {dy_dt}\nangle: {path_angle * 180 / np.pi}")

        # Lateral error: how far lookahead point is from image center
        lateral_error = np.clip(
            (lookahead_x - self.width / 2) / (self.width / 2),
            -1, 1
        )

        # Yaw rate: combine lateral error and path angle
        path_angle_normalized = path_angle / (np.pi / 2)  # normalize to [-1, 1]
        yaw_rate = np.clip(
            lateral_error * 0.2 + path_angle_normalized * 0.7,
            -1, 1
        )

        # ========================
        # ===== Visualization ====
        # ========================
        frame_debug = self.frame_debug.copy()

        # Draw ROI window (GRAY)
        cv2.rectangle(frame_debug, (x_start, 0), (x_end, self.height), (100, 100, 100), 1)

        # Draw sampled points (RED)
        for x, y in zip(cols_all, rows_all):
            cv2.circle(frame_debug, (int(x), int(y)), 6, (0, 0, 255), -1)

        # Draw parametric curve (BLUE)
        t_vals = np.linspace(0, float(len(cols_all) - 1), 100)
        x_vals = np.clip(np.polyval(coeffs_x, t_vals), 0, self.width - 1)
        y_vals = np.clip(np.polyval(coeffs_y, t_vals), 0, self.height - 1)
        for i in range(len(t_vals) - 1):
            x1, y1 = int(x_vals[i]),   int(y_vals[i])
            x2, y2 = int(x_vals[i+1]), int(y_vals[i+1])
            cv2.line(frame_debug, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # Draw lookahead point (GREEN)
        cv2.circle(frame_debug, (int(lookahead_x), int(lookahead_y)), 8, (0, 255, 0), -1)

        # Draw path direction arrow at lookahead (YELLOW)
        arrow_length = 50
        norm = np.hypot(dx_dt, dy_dt)
        if norm > 0:
            adx = int(arrow_length * dx_dt / norm)
            ady = int(arrow_length * dy_dt / norm)
        else:
            adx, ady = 0, -arrow_length
        cv2.arrowedLine(
            frame_debug,
            (int(lookahead_x), int(lookahead_y)),
            (int(lookahead_x + adx), int(lookahead_y + ady)),
            (0, 255, 255), 2
        )

        # Draw image center line (CYAN)
        cv2.line(frame_debug, (self.width // 2, 0), (self.width // 2, self.height), (0, 255, 255), 1)

        self.frame_debug = frame_debug

        return lateral_error, yaw_rate, np.degrees(path_angle)