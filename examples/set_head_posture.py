
import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False


yaw = 0.0
pitch = 0.2

with pepper_interface.get(IP,PORT,simulation,stop=False) as pepper:
    
    pepper.head.set(yaw,pitch)

    time.sleep(3.0)
