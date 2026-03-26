import time
import numpy as np
from pymavlink import mavutil
from config import IMG_WIDTH, IMG_HEIGHT

class DroneController:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.master = None
        self.home = {"x": 0, "y": 0, "z": 0}
        self.gain = 0.2
        self.lateral_gain = 2.5
        self.center_threshold = 0.1
        self.max_velocity = 0.3

    def connect(self):
        print("Connecting to SITL...", flush=True)
        self.master = mavutil.mavlink_connection(self.connection_str)

    def wait_heartbeat(self):
        print("Waiting for heartbeat...", flush=True)
        self.master.wait_heartbeat()
        print(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}", flush=True)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            75000,
            0, 0, 0, 0, 0
        )

    def set_mode(self, mode="GUIDED"):
        print(f"Setting mode to {mode}...", flush=True)
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(2)

    def arm(self):
        print("Arming vehicle...", flush=True)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            21196,
            0, 0, 0, 0, 0
        )
        time.sleep(3)

    def takeoff(self, altitude=1.0):
        print(f"Taking off to {altitude} m...", flush=True)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            altitude
        )
        time.sleep(5)

    def land(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0,
            0
        )
        print("Landing...", flush=True)
        time.sleep(15)

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            21196,
            0, 0, 0, 0, 0
        )
        print("Disarming...", flush=True)

    def return_to_home(self):
        print("Returning to home...", flush=True)
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int("010111111000", 2),
            self.home["x"], self.home["y"], self.home["z"],
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def send_velocity(self, vx=0, vy=0, vz=0, yaw_rate=0):
        self.wait_attitude()
        velocity_mask = int("110111000111", 2)
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            velocity_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate
        )
        self.master.mav.send(msg)

    def center_on_tag(self, error):
        if error is None:
            return None

        error_x, error_y = error
        self.send_velocity(
            vx=np.clip(-error_y * self.gain, -self.max_velocity/5, self.max_velocity/5),
            vy=np.clip(error_x * self.gain, -self.max_velocity/5, self.max_velocity/5)
        )
        return abs(error_x) < self.center_threshold and abs(error_y) < self.center_threshold

    def turn(self, angle, speed=20):
        direction = 1 if angle >= 0 else -1
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            abs(angle),
            speed,
            direction,
            1,
            0, 0, 0
        )

    def turn_to_direction(self, direction=None, angle=None):
        if angle is None:
            angle = np.degrees(np.arctan2(direction[1], direction[0])) * -1
        self.turn(angle)
        print(f"Turning to direction {direction} (angle {angle:.1f} degrees)", flush=True)
        time.sleep(8)

    def follow_path(self, steering_error, yaw_rate, path_angle, speed=0.5):
        if steering_error is None:
            self.send_velocity()
            return False
        if abs(path_angle) > 30:
            print(path_angle)
            self.turn(path_angle * 2 / 3, speed=8)
            time.sleep(2)
        self.send_velocity(
            vx=speed,
            vy=np.clip(steering_error * speed * self.lateral_gain, -1, 1),
            yaw_rate=yaw_rate
        )
        return True

    def wait_attitude(self):
        self.master.recv_match(type='ATTITUDE', blocking=True)