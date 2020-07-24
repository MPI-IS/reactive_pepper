import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False


with pepper_interface.get(IP,PORT,simulation) as pepper:

    left_joints = pepper.arms.get_joints(True)
    right_joints = pepper.arms.get_joints(False)

    left_posture = {joint:0.0 for joint in left_joints}
    right_posture = {joint:0.0 for joint in right_joints}

    pepper.arms.set(True,left_posture,velocity=0.4)
    pepper.arms.set(False,right_posture,velocity=0.4)

    # waiting while left arm active
    while pepper.arms.running(True):
        time.sleep(0.01)
    # waiting while right arm active
    while pepper.arms.running(False):
        time.sleep(0.01)

    
        
    
