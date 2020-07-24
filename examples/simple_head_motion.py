import math,time,random
import pepper_interface


IP = "192.168.0.147"
PORT = 9559
simulation = False


def _get_angle():
    return random.uniform(-math.pi/4.0, +math.pi/4.0)

def _get_velocity():
    return random.uniform(0.01, 0.02)

time_switch = 0.5


with pepper_interface.get(IP,PORT,simulation) as pepper:

    time_start = time.time()
    motion_start = time.time()-time_switch
    
    while time.time() - time_start < 15:

        if time.time()>(motion_start+time_switch):

            yaw = _get_angle()
            pitch = _get_angle()
            velocity = _get_velocity()

            motion_start = time.time()
            
            pepper.head.set(yaw,pitch,velocity)
            
        time.sleep(0.01)
            

