# tutorial to create a python module:
# http://doc.aldebaran.com/2-1/dev/python/reacting_to_events.html#python-reacting-to-events
# http://doc.aldebaran.com/2-1/dev/python/running_python_code_on_the_robot.html?highlight=autoload

# or

# http://doc.aldebaran.com/2-5/dev/libqi/guide/py-service.html?highlight=onbang
# and
# http://doc.aldebaran.com/2-5/dev/tutos/create_a_new_service.html?highlight=outside%20choregraphe


# -*- encoding: UTF-8 -*-
""" Say 'hello, you' each time a human face is detected

"""

import sys
import time
import threading
import math

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

NAO_IP = "nao.local"


# Global variable to store the LookAt module instance
LookAt = None


class _lam :

    _yaw = None
    _pitch = None
    _velocity = 0.2
    _ramping = math.pi/4.0
    _lock = threading.Lock()
    _motion = None
    _mid_yaw = None
    _mid_pitch = None
    _running = False
    
    @classmethod
    def set(cls,yaw,pitch,velocity):
        with cls._lock:
            cls._yaw = yaw
            cls._pitch = pitch
            cls._velocity = velocity

    @classmethod
    def _get_speed(cls,desired_yaw,desired_pitch):
        
        values = cls.motion.getAngles(["HeadYaw","HeadPitch"],True)
        current_yaw,current_pitch = values["HeadYaw"],values["HeadPitch"]

        if desired_yaw is None :
            return None,None
        

        yaw = (current_yaw,desired_yaw)
        pitch = (current_pitch,desired_pitch)

        velocity = [None,None]

        with cls._lock:
            for index,current,desired in enumerate(yaw,pitch):
                diff = abs(current-desired)
                if diff > cls._ramping :
                    velocity[index]=cls._velocity
                else :
                    velocity[index] = cls._velocity * diff / cls._ramping

        return velocity

    
    @classmethod
    def _update(cls):

        with cls._lock:
            desired_yaw,desired_pitch = cls._yaw,cls._pitch

        v_yaw,v_pitch = cls._get_speed(desired_yaw,desired_pitch)

        mid = self._motion_proxy.post.setAngles(["HeadYaw"], [desired_yaw], v_yaw)
        try :
            cls._motion.stop(cls._mid_yaw)
        except :
            pass
        self._mid_yaw = mid

        mid = self._motion_proxy.post.setAngles(["HeadPitch"], [desired_pitch], v_pitch)
        try :
            cls._motion.stop(cls._mid_pitch)
        except :
            pass
        self._mid_pitch = mid

        
    @classmethod
    def run(cls):

        cls._running = True

        while cls._running:

            cls._update()
            time.sleep(0.05)

            
    @classmethod
    def stop(cls):

        cls._running=False
            
            
class LookAtModule(ALModule):
    """ LookAt
    """
    def __init__(self, name):

        ALModule.__init__(self, name)
        _lam._motion = ALProxy("ALMotion")
        self._thread = None

    def start(self):
        """ start """
        self._thread = threading.Thread(target=_lam.run) 
        self._thread.setDaemon(True)
        self._thread.start()

    def stop(self):
        """ stop """
        if self._thread is not None:
            _lam.stop()
            self._thread.join()
            self._thread=None
        
    def set(self,desired_yaw,desired_pitch,velocity):
        """ yaw and pitch """
        _lam.set(desired_yaw,desired_pitch,velocity)
        
        

def main():
    """ Main entry point

    """
    parser = OptionParser()
    parser.add_option("--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
    parser.add_option("--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")
    parser.set_defaults(
        pip=NAO_IP,
        pport=9559)

    (opts, args_) = parser.parse_args()
    pip   = opts.pip
    pport = opts.pport

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("myBroker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       pip,         # parent broker IP
       pport)       # parent broker port


    # Warning: HumanGreeter must be a global variable
    # The name given to the constructor must be the name of the
    # variable
    global LookAt
    LookAt = LookAtModule("LookAt")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        LookAt.stop()
        myBroker.shutdown()
        sys.exit(0)

    LookAt.stop()



if __name__ == "__main__":
    main()



