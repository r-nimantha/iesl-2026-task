import time
from pymavlink import mavutil

class DroneController:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.master = None
        self.home = {"x": 0, "y": 0, "z": 0}

    def connect(self):
        print("Connecting to SITL...")
        self.master = mavutil.mavlink_connection(self.connection_str)

    def wait_heartbeat(self):
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}")

    def set_mode(self, mode="GUIDED"):
        print(f"Setting mode to {mode}...")
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(2)

    def arm(self):
        print("Arming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            21196,
            0,0,0,0,0
        )
        time.sleep(3)

    def takeoff(self, altitude=2.0):
        print(f"Taking off to {altitude} m...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,0,0,0,
            0,0,
            altitude
        )
        time.sleep(5)
        # msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # self.home["x"] = msg.x
        # self.home["y"] = msg.y
        # self.home["z"] = msg.z

    def return_to_home(self):
        print("Returning to home...")
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int("010111111000", 2),
            self.home["x"], self.home["y"], self.home["z"],
            0,0,0,
            0,0,0,
            0,0
        )

    def send_velocity(self, vx=0, vy=0, vz=0, duration=5.0, rate=10):
        """
        Sends velocity in body frame continuously for `duration` seconds.
        `rate` = messages per second.
        """
        velocity_mask = int("110111000111", 2)
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            velocity_mask,
            0,0,0,
            vx,vy,vz,
            0,0,0,
            0,0
        )

        interval = 1.0 / rate
        end_time = time.time() + duration
        while time.time() < end_time:
            self.master.mav.send(msg)
            time.sleep(interval)

    def send_velocity_once(self, vx=0, vy=0, vz=0, yaw_rate=0, rate=10):
        """
        Sends a single velocity command.
        """
        velocity_mask = int("110111000111", 2)
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            velocity_mask,
            0,0,0,
            vx,vy,vz,
            0,0,0,
            0,yaw_rate
        )
        self.master.mav.send(msg)



    def follow_line(self, camera, forward_speed=0.5, lateral_gain=1.0, heading_gain=0.5, update_rate=10):
        print("Starting line following...")
        interval = 1.0 / update_rate

        while True:

            # AprilTag priority
            tag_found, tag_id, tag_cx, tag_cy = camera.get_april_tag_info()
            if tag_found:
                print(f"April tag {tag_id} detected! Mission complete.")
                self.send_velocity_once(0, 0, 0)
                return True

            # Line detection
            line_found, line_cx, line_cy, line_angle = camera.get_line_info()
            if not line_found:
                print("Line lost!")
                self.send_velocity_once(0, 0, 0)
                return False

            image_center_x = 320

            deviation = (line_cx - image_center_x) / image_center_x

            vy = deviation * lateral_gain

            yaw_rate = 0
            if line_angle is not None:
                angle_error = line_angle
                yaw_rate = -angle_error * heading_gain

            vx = forward_speed * (1 - min(abs(deviation), 0.7))

            self.send_velocity_once(
                vx=vx,
                vy=vy,
                vz=0,
                yaw_rate=yaw_rate
            )

            time.sleep(interval)


