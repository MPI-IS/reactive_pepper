import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False

with pepper_interface.get(IP,PORT,simulation,stop=False) as pepper:

    # closing
    pepper.hands.set(True,0.4,velocity=0.5) # left
    pepper.hands.set(False,0.4,velocity=0.5) # right

    while pepper.hands.running(True) or pepper.hands.running(False):
        time.sleep(0.1)
