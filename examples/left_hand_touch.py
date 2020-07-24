import math,time,random
import pepper_interface

IP = "192.168.0.147"
PORT = 9559
simulation = False



with pepper_interface.get(IP,PORT,simulation) as pepper:

    time_start = time.time()

    while time.time()-time_start < 10:
    
        data = pepper.left_hand_touch.get()

        if data is not None:

            data,time_stamp = data

            print data,time_stamp
            
        time.sleep(0.2)

    
