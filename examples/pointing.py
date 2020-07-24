import math,time,random
import pepper_interface



IP = "192.168.0.147"
PORT = 9559
simulation = False


# this example requires to have playful kinematics installed
# and sourced
# see: https://github.com/vincentberenz/playful_kinematics


with pepper_interface.get(IP,PORT,simulation) as pepper:

    time_start = time.time()

    x = 2.0
    p = 0.0
    
    while time.time() - time_start < 15:

        p+=0.04
        
        # position in frame relative
        # to the robot
        position = [x,
                    0.5*math.cos(p),
                    1.2+0.5*math.sin(p)]

        # position in absolute frame
        position = pepper.transforms.in_reference_frame(position)

        left_arm = True
        hand_orientation = [-math.pi/2.0,None,None]
        posture = pepper.arms.pointing(left_arm,
                                       position,
                                       hand_orientation=hand_orientation)
        if posture :
            pepper.arms.set(left_arm,posture,velocity=0.4)
        
        time.sleep(0.01)
                             
