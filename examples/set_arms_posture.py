
import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False


posture = { 'RShoulderRoll': -0.33133983612060547,
            'RWristYaw': 0.8620660305023193,
            'RElbowRoll': 0.9035146236419678,
            'RElbowYaw': 0.5077476501464844,
            'RShoulderPitch': 0.38042736053466797 }


with pepper_interface.get(IP,PORT,simulation,stop=False) as pepper:

    target_time = time.time()+5.0
    
    pepper.arms.set(False,posture,target_time=target_time)

    time.sleep(6.0)
    


