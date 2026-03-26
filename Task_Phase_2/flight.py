from camera import CameraStream
from control import DroneController
import time
import threading
import random
import numpy as np

Airports = [1, 2]
STABLE_FRAMES_REQUIRED = 8
MAX_TRIES_TO_CENTER = 1000
MAX_VELOCITY = 0.1
nodes = {}

def decode_tag(tag_id):
    country = tag_id // 100
    status = (tag_id // 10) % 10
    num_paths = tag_id % 10
    return country, status, num_paths

def fly_to_next_airport(cam, drone, current_id=None):
    stable_count = 0
    tries_to_center = 0

    while True:
        tag = cam.detect_tag()
        if tag and (current_id is None or tag.tag_id != current_id):
            error = cam.get_tag_error()
            centered = drone.center_on_tag(error)
            if centered:
                stable_count += 1
                if stable_count >= STABLE_FRAMES_REQUIRED:
                    drone.send_velocity()
                    return tag.tag_id
            else:
                stable_count = 0
                tries_to_center += 1
                if tries_to_center > MAX_TRIES_TO_CENTER:
                    tries_to_center = 0
                    print("Waiting for tag to stabilize...", flush=True)
                    for _ in range(20):
                        drone.send_velocity()
                        time.sleep(1)

        else:
            steering_error, yaw_rate, path_angle = cam.compute_steering_debug()
            drone.follow_path(steering_error, yaw_rate, path_angle, speed=MAX_VELOCITY)


if __name__ == "__main__":
    cam = CameraStream()
    drone = DroneController('tcp:localhost:5763')

    cam_thread = threading.Thread(target=cam.run, daemon=True, name="CameraThread")
    cam_thread.start()

    drone.connect()
    drone.wait_heartbeat()
    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(altitude=1.5)

    valid_airports = [a for a in Airports if a != 0]
    current_id = None

    while True:
        current_id = fly_to_next_airport(cam, drone, current_id=current_id)
        country, status, num_paths = decode_tag(current_id)
        print(f"Arrived at airport: country={country}, status={status}, paths={num_paths}", flush=True)

        if country in valid_airports and status == 1:
            valid_airports.remove(country)
            print(f"Found target country {country}, landing...", flush=True)
            drone.land()

            if not valid_airports:
                print("Mission complete!", flush=True)
                break
            
            drone.set_mode("GUIDED")
            drone.arm()
            drone.takeoff(altitude=1.7)
            current_id = None
            continue

        tag = cam.detect_tag()
        if tag is None:
            continue


        direction_angles = cam.get_path_angles()
        print(f"Detected path angles: {direction_angles}", flush=True)
        if direction_angles:
            while True:
                chosen_angle = random.choice(direction_angles)
                if abs(chosen_angle) < 135 and abs(chosen_angle) > 30:
                    break
            print(f"Exploring path angle: {chosen_angle:.2f}", flush=True)
            drone.turn_to_direction(angle=chosen_angle)