import math,time,random
import pepper_interface



IP = "192.168.0.147"
PORT = 9559
simulation = False

publish = True

with pepper_interface.get(IP,PORT,simulation,publish=publish) as pepper:

    time.sleep(60)
