import threading
import time
from control import DroneController
from camera import CameraStream

if __name__ == "__main__":
    cam = CameraStream(host="localhost", port=5599, capture_count=10, interval=1.0)
    cam_thread = threading.Thread(target=cam.run, daemon=True, name="CameraThread")
    cam_thread.start()
    time.sleep(2)

    drone = DroneController(connection_str="tcp:localhost:5763")
    drone.connect()
    drone.wait_heartbeat()
    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(altitude=2.0)
    time.sleep(2)

    drone.send_velocity(0.5, 0, 0, duration=1)
    time.sleep(5)
    
    # Find the April tag

    print("Starting mission...")
    tag_found = drone.follow_line(
        fine = False,
        camera=cam,
        forward_speed=0.2,
        lateral_gain=0.2,
        heading_gain=0.2,
        update_rate=10
    )
    
    if tag_found:
        print("Performing 90 degree turn to the right...")
    else:
        print("Line lost. Returning to home...")
        drone.set_mode("RTL")

    # Make a 90 degree turn to the right
    drone.turn_right_90_degrees()
    time.sleep(15)
    drone.send_velocity(0.3, 0, 0, duration=5)
    time.sleep(5)

    # Find the landing pad (April tag)
    tag_found = drone.follow_line(
        fine = True,
        camera=cam,
        forward_speed=0.2,
        lateral_gain=0.2,
        heading_gain=0.2,
        update_rate=10
    )

    if tag_found:
        print("Landing pad found!")
        drone.land()
    else:
        print("Line lost. Returning to home...")
        drone.set_mode("RTL")
    
    time.sleep(10)
    print("Mission complete.")

