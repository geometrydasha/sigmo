import cv2
import numpy as np
from pioneer_sdk import Camera
from pioneer_sdk import Pioneer
import time
import threading

# Connect to the drone camera
d1 = Pioneer(ip='127.0.0.1', mavlink_port='8000', logger=True)
d2 = Pioneer(ip='127.0.0.1', mavlink_port='8001', logger=True)
camera = Camera(timeout=0.1, ip='127.0.0.1', port=18000, video_buffer_size=65000, log_connection=True)
drones = [d1, d2]
x = -5
x2 = 5
y = -5
home1 = [-0.85, -4.91]
home2 = [0.31, -4.92]
d1.arm()
d2.arm()
time.sleep(2)
d1.takeoff()
d2.takeoff()
d1.go_to_local_point(-5, -5, 1.5, 0)
d2.go_to_local_point(5, -5, 1, 0)
while not d2.point_reached():
    time.sleep(0.03)
d1.go_to_local_point(-5, 5, 1.5, 0)
d2.go_to_local_point(5, 5, 1, 0)
while not d2.point_reached():
    time.sleep(0.03)


def moving1(xd, d, home):
    while True:
        d.go_to_local_point(xd, 5, 1.5, 0)
        while not d.point_reached():
            time.sleep(0.03)
        xd += 1
        d.go_to_local_point(xd, 5, 1.5, 0)
        while not d.point_reached():
            time.sleep(0.03)
        d.go_to_local_point(xd, -5, 1.5, 0)
        while not d.point_reached():
            time.sleep(0.03)
        xd += 1
        d.go_to_local_point(xd, -5, 1.5, 0)
        while not d.point_reached():
            time.sleep(0.03)
        if xd >= 5:
            d.go_to_local_point(home[0], home[1], 1.5, 0)
            while not d.point_reached():
                time.sleep(0.03)
            d.land()
            d.disarm()
            break


def moving2(yd, d, home):
    while True:
        d.go_to_local_point(5, yd, 1, 0)
        while not d.point_reached():
            time.sleep(0.03)
        yd -= 1
        d.go_to_local_point(5, yd, 1, 0)
        while not d.point_reached():
            time.sleep(0.03)
        d.go_to_local_point(-5, yd, 1, 0)
        while not d.point_reached():
            time.sleep(0.03)
        yd -= 1
        d.go_to_local_point(-5, yd, 1, 0)
        while not d.point_reached():
            time.sleep(0.03)
        if yd <= -5:
            d.go_to_local_point(home[0], home[1], 1, 0)
            while not d.point_reached():
                time.sleep(0.03)
            d.land()
            d.disarm()
            break


if __name__ == "__main__":
    def sigmo():
        while True:
            raw_frame = camera.get_frame()  # Get raw data
            if raw_frame is not None:
                # Decode data to get image
                frame = cv2.imdecode(
                    np.frombuffer(raw_frame, dtype=np.uint8), cv2.IMREAD_COLOR
                )
            cv2.imshow("video", frame)  # Show an image on the screen

            if cv2.waitKey(1) == 27:  # Exit if the ESC key is pressed
                break

        cv2.destroyAllWindows()  # Close all opened openCV windows

    t = threading.Thread(target=moving1, args=(x, d1, home1))
    t2 = threading.Thread(target=moving2, args=(x2, d2, home2))
    t3 = threading.Thread(target=sigmo, args=())

    t.start()
    t2.start()
    t3.start()

    t.join()
    t2.join()
    t3.join()