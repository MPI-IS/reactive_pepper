import math,time,random
import pepper_interface

IP = "127.0.0.1"
PORT = 36195
simulation = True



with pepper_interface.get(IP,PORT,simulation) as pepper:

    position = [1.0,-1.0,0.5]
    position = pepper.transforms.in_reference_frame(position)

    time_start = time.time()

    while time.time() - time_start < 5 :

        pepper.head.look_at(position,velocity=0.1)
    
        time.sleep(0.02)
