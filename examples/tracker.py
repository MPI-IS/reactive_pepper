import math,time,random
import pepper_interface


IP = "192.168.0.147"
PORT = 9559
simulation = True



with pepper_interface.get(IP,PORT,simulation) as pepper:

    pepper.tracker.start()
    
    time_start = time.time()

    x = 2.0
    p = 0.0

    while time.time()-time_start < 30.0 :

            p+=0.04

            # position in frame relative
            # to the robot
            position = [x,
                        2.0*math.cos(p),
                        1.2+0.5*math.sin(p)]

            # position in absolute frame
            position = pepper.transforms.in_reference_frame(position)

            if position: # None may have been returned at startup
                pepper.tracker.set(position)

            time.sleep(0.1)

        
    pepper.tracker.stop()

    
