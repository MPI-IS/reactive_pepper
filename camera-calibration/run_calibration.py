import math,time,random
import pepper_interface
import sys

IP = "192.168.0.147"
PORT = 9559
simulation = False

# this will make sure camera ir is published
camera_calibration=True
publish = True

with pepper_interface.get(IP,PORT,
                          simulation,
                          publish=publish,
                          camera_calibration=camera_calibration) as pepper:

    running = True
    
    while running:
        try :
            time.sleep(0.1)
        except KeyboardInterrupt :
            running=False
            
