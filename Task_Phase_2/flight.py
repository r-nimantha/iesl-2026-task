from camera import CameraStream
from control import DroneController
import time
import threading
import random
import numpy as np

Airports = [1, 2]
STABLE_FRAMES_REQUIRED = 8
MAX_TRIES_TO_CENTER = 120
LOOP_RATE = 20
loop_dt = 1.0 / LOOP_RATE
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
        t_start = time.time()

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

            print(f"Stable count: {stable_count}", flush=True)

        else:
            steering_error, yaw_rate, path_angle = cam.compute_steering_debug()
            drone.follow_path(steering_error, yaw_rate, path_angle, speed=0.1)

        elapsed = time.time() - t_start
        sleep_time = loop_dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    cam = CameraStream()
    drone = DroneController('tcp:localhost:5763')

    cam_thread = threading.Thread(target=cam.run, daemon=True, name="CameraThread")
    cam_thread.start()

    drone.connect()
    drone.wait_heartbeat()
    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(altitude=1.7)

    target_idx = 0
    current_id = None

    while target_idx < len(Airports) and Airports[target_idx] != 0:
        t_start = time.time()

        current_id = fly_to_next_airport(cam, drone, current_id=current_id)
        country, status, num_paths = decode_tag(current_id)
        print(f"Arrived at airport: country={country}, status={status}, paths={num_paths}", flush=True)

        if country == Airports[target_idx] and status == 1:
            print(f"Found target country {Airports[target_idx]}, landing...", flush=True)
            drone.land()
            time.sleep(5)

            target_idx += 1
            if target_idx >= len(Airports) or Airports[target_idx] == 0:
                print("Mission complete!", flush=True)
                break

            drone.takeoff(altitude=1.7)
            current_id = None
            continue

        tag = cam.detect_tag()
        if tag is None:
            continue

        # path detection should come here, but for now just turn randomly.
        next_dir = random.choice([(1, 0), (0, 1), (0, -1)])
        print(f"Exploring path direction: {next_dir[0]}", flush=True)
        drone.turn_to_direction(next_dir)

        elapsed = time.time() - t_start
        sleep_time = loop_dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)