import time
from pymavlink import mavutil

IMG_WIDTH =  640
IMG_HEIGHT = 480

class DroneController:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.master = None
        self.home = {"x": 0, "y": 0, "z": 0}

    def connect(self):
        print("Connecting to SITL...", flush=True)
        self.master = mavutil.mavlink_connection(self.connection_str)

    def wait_heartbeat(self):
        print("Waiting for heartbeat...", flush=True)
        self.master.wait_heartbeat()
        print(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}", flush=True)

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
            0,0,0,0,0
        )
        time.sleep(3)

    def takeoff(self, altitude=2.0):
        print(f"Taking off to {altitude} m...", flush=True)
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

    def land(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,0,0,0,
            0,0,
            0
        )
        print("Landing...", flush=True)
        # disarm after landing
        time.sleep(10)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            21196,
            0,0,0,0,0
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
            0,0,0,
            0,0,0,
            0,0
        )

    def send_velocity(self, vx=0, vy=0, vz=0, duration=5.0, rate=10):
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
        self.master.mav.send(msg)


    def center_tag(self, tag_cx, tag_cy):
        error_x = tag_cx - IMG_WIDTH/2
        error_y = tag_cy - IMG_HEIGHT/2
        gain = 0.0002
        vx = gain * error_y * -1
        vy = gain * error_x
        vz = 0
        # print(f"[CENTER_TAG] Tag center: ({tag_cx:.2f}, {tag_cy:.2f})")
        self.send_velocity_once(vx=vx, vy=vy, vz=vz)


    def turn_right_90_degrees(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            90,
            10,
            1,1,
            0,0,0
        )


    def follow_line(self, fine, camera, forward_speed=0.5, lateral_gain=1.0, heading_gain=0.5, update_rate=10):
        id_displayed = False
        if fine:
            x_tolerance = 20
            y_tolerance = 20
        else:
            x_tolerance = 50
            y_tolerance = 30

        while True:
            tag_found, tag_id, tag_cx, tag_cy = camera.get_april_tag_info()
            if tag_found:
                if not id_displayed:
                    print(f"April tag {tag_id} detected", flush=True)
                    id_displayed = True
                self.center_tag(tag_cx, tag_cy)
                if abs(tag_cx - IMG_WIDTH/2) < x_tolerance and abs(tag_cy - IMG_HEIGHT/2) < y_tolerance:
                    print(f"April tag is centered", flush=True)
                    self.send_velocity(0, 0, 0, 3)
                    time.sleep(3)
                    camera.reset_detections()
                    return True
                continue

            line_found, line_cx, line_cy, line_angle = camera.get_line_info()
            if not line_found:
                print(f"Line lost!", flush=True)
                self.send_velocity_once(0, 0, 0)
                return False

            deviation = (line_cx - IMG_WIDTH/2) / (IMG_WIDTH/2)
            vy = deviation * lateral_gain
            vx = forward_speed * (1 - min(abs(deviation), 0.7))

            self.send_velocity_once(
                vx=vx,
                vy=vy,
                vz=0,
            )

            time.sleep(1.0 / update_rate)


