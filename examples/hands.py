import math,time,random
import pepper_interface

IP = "127.0.0.1"
PORT = 36417
simulation = False

with pepper_interface.get(IP,PORT,simulation) as pepper:

    # opening
    pepper.hands.set(True,1.0,velocity=0.5) # left
    pepper.hands.set(False,1.0,velocity=0.5) # right

    while pepper.hands.running(True) or pepper.hands.running(False):
        time.sleep(0.1)

    # closing
    pepper.hands.set(True,0.0,velocity=0.5) # left
    pepper.hands.set(False,0.0,velocity=0.5) # right

    while pepper.hands.running(True) or pepper.hands.running(False):
        time.sleep(0.1)

