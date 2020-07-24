import math,time,random
import pepper_interface


IP = "192.168.0.147"
PORT = 9559
simulation = False


position1 = {'LShoulderPitch': 1.115204095840454, 'LShoulderRoll': 1.4278449058532715,
             'LElbowYaw': -0.012271881103515625, 'LElbowRoll':-1.2670681476593018,
             'LWristYaw':0.5920820236206055}

position2 = {'LShoulderPitch': 0.0981748104095459, 'LShoulderRoll':  1.418932318687439,
             'LElbowYaw': -1.920543909072876, 'LElbowRoll': -1.3299614191055298,
             'LWristYaw': 0.6963939666748047}


with pepper_interface.get(IP,PORT,simulation) as pepper:

    for _ in range(3):

        pepper.arms.set(False,pepper.arms.symmetry.left_to_right(position1),
                        velocity=0.7)
        pepper.arms.set(True,position1,
                        velocity=0.7)

        while pepper.arms.running(True):
            time.sleep(0.01)

        pepper.arms.set(True,position2,
                        velocity=0.7)
        pepper.arms.set(False,pepper.arms.symmetry.left_to_right(position2),
                        velocity=0.7)

        while pepper.arms.running(True):
            time.sleep(0.01)
        
