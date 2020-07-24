import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False

with pepper_interface.get(IP,PORT,simulation) as pepper:

    target_time = time.time()+5.0
    
    pepper.arms.set(True,{"LShoulderRoll":1.2},target_time=target_time)

    time.sleep(2.5)
    
    pepper.arms.set(False,{"RShoulderRoll":-1.2},target_time=target_time)

    time.sleep(4.0)
    
    
    # note: this type of sync will only work with the arms.
    #       for the head, hip, and knee, this will not work
    #       (for some reason, could not get motion proxy angle interpolation
    #       to run on these other body parts)
    
