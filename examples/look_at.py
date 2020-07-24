import math,time,random
import pepper_interface


IP = "127.0.0.1"
PORT = 36195
simulation = True


with pepper_interface.get(IP,PORT,simulation) as pepper:

    time_start = time.time()

    x = 2.0
    p = 0.0

    while time.time() - time_start < 15:

        p+=0.02
        
        # position in frame relative
        # to the robot
        position = [x,
                    0.5*math.cos(p),
                    1.2+0.5*math.sin(p)]

        # position in absolute frame
        position = pepper.transforms.in_reference_frame(position)

        if position: # None may have been returned at startup
            pepper.head.look_at(position,velocity=0.2)

        time.sleep(0.01)
