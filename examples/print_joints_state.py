import math,time,random
import pepper_interface

IP = "127.0.0.1"
PORT = 40363
simulation = False

with pepper_interface.get(IP,PORT,simulation) as pepper:

    print repr(pepper.get_joints_values())
