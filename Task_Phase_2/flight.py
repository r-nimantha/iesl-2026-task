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
forbidden_paths = []

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
                    for _ in range(20):
                        drone.send_velocity()
                        time.sleep(1)

        else:
            steering_error, yaw_rate, path_angle = cam.compute_steering()
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
    drone.takeoff(altitude=1.7)

    valid_airports = [a for a in Airports if a != 0]
    current_id = None
    visited = []
    node_paths = {}
    stack = []

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
            time.sleep(4)
            drone.set_mode("GUIDED")
            drone.arm()
            drone.takeoff(altitude=1.7)
            time.sleep(2)
            current_id = None
            continue

        if current_id in visited:
            if not stack:
                print("Exhausted all paths!", flush=True)
                break
            _, back_angle = stack.pop()
            #print(f"Already visited {current_id}, backtracking via angle {back_angle:.1f}", flush=True)
            drone.turn_to_direction(angle=back_angle)
            current_id = None
            continue

        visited.append(current_id)

        paths = cam.get_path_angles()
        if len(paths) == 0:
            print("No paths found, backtracking...", flush=True)
            if not stack:
                break
            _, back_angle = stack.pop()
            drone.turn_to_direction(angle=back_angle)
            current_id = None
            continue

        arriving_path = min(paths, key=lambda p: abs(abs(p[0]) - 180))
        back_angle = arriving_path[0]
        # print(f"Arriving path angle: {back_angle:.1f}", flush=True)

        if len(forbidden_paths) == 0:
            forbidden_paths.append((current_id, arriving_path[1]))

        if current_id not in node_paths:
            node_paths[current_id] = [
                p for p in paths
                if p is not arriving_path
                and (current_id, p[1]) not in forbidden_paths
            ]

        if not node_paths[current_id]:
            print("Dead end, backtracking...", flush=True)
            if not stack:
                break
            _, back_angle = stack.pop()
            drone.turn_to_direction(angle=back_angle)
            current_id = None
            continue

        # print("Detected paths:", flush=True)
        # for angle, side_name in paths:
        #     if (current_id, side_name) in forbidden_paths:
        #         print(f"  {side_name}: {angle:.2f} degrees (forbidden)", flush=True)
        #     else:
        #         print(f"  {side_name}: {angle:.2f} degrees", flush=True)

        stack.append((current_id, back_angle))
        # chosen_angle, chosen_side = random.choice(node_paths[current_id])

        # just to have the best path for the video.
        if len(node_paths[current_id]) > 1:
            chosen_angle, chosen_side = node_paths[current_id][1]
        else:
            chosen_angle, chosen_side = node_paths[current_id][0]

        node_paths[current_id] = [
            p for p in node_paths[current_id]
            if p[1] != chosen_side
        ]
        # print(f"Exploring: angle={chosen_angle:.1f}, side={chosen_side}", flush=True)
        drone.turn_to_direction(angle=chosen_angle)