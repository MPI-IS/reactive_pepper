import math,time,random
import pepper_interface


IP = "192.168.0.147"
PORT = 9559
simulation = False


with pepper_interface.get(IP,PORT,simulation) as pepper:

    v = 0

    time_start = time.time()

    while time.time()-time_start < 15 :

        v+=0.005

        x = 0.3*math.sin(v)
        y = 0.3*math.cos(v)
        theta = 0.3*math.sin(v)

        pepper.wheels.set(x,y,theta)

        sensors,time_stamp = pepper.wheels_sensor.get()
        print("\n")
        print("time stamp:",time_stamp)
        for wheel in sensors.values():
            print(wheel)
        
        time.sleep(0.01)
