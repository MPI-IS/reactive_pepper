import math,time,random
import pepper_interface



IP = "192.168.0.147"
PORT = 9559
simulation = False


with pepper_interface.get(IP,PORT,simulation) as pepper:

    pepper.arms.set(True,{"LShoulderRoll":0.3})
    pepper.arms.set(False,{"RShoulderRoll":-0.3})

    x = 0

    time_start = time.time()
    
    while time.time() - time_start < 20:

        x+=0.24

        pitch = math.sin(x) / (2.2*math.pi)

        pepper.arms.set( True,
                        {"LShoulderRoll":0.3-pitch,
                         "LElbowRoll":-4.0*pitch,
                         "LElbowYaw":-3.0*pitch} )


        pepper.arms.set(False,{"RShoulderRoll":-0.3-pitch,
                               "RElbowRoll":-4.0*pitch,
                               "RElbowYaw":-3.0*pitch})
            
        pepper.hip.set(pitch,pitch/2.0,velocity=0.1)
        pepper.knee.set(-pitch/2.0,velocity=0.1)
        
        time.sleep(0.05)

