import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False


with pepper_interface.get(IP,PORT,simulation) as pepper:

    time.sleep(1.0)
    print
    print pepper.battery.get()
    print
