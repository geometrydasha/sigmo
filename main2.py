from pioneer_sdk import Pioneer
import time
import threading

d1 = Pioneer(ip='127.0.0.1', mavlink_port='8000', logger=True)
d2 = Pioneer(ip='127.0.0.1', mavlink_port='8001', logger=True)
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

t = threading.Thread(target=moving1, args=(x, d1, home1))
t2 = threading.Thread(target=moving2, args=(x2, d2, home2))

t.start()
t2.start()

t.join()
t2.join()

#threads = []
#for i in range(2):
#    t = threading.Thread(target=moving1, args=(x, d1, home1))
#    threads.append(t)
#    t.start()