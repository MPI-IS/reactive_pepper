import threading,copy

class Wheels:

    
    def __init__(self,motion_proxy):
        self._motion = motion_proxy
        self._current = [0,0,0]
        self._lock = threading.Lock()


    def get(self):
        with self._lock:
            return copy.deepcopy(self._current)
        
    def set(self,x,y,theta):

        with self._lock:
            speed = copy.deepcopy(self._current)

        if x is not None:
            speed[0] = x

        if y is not None:
            speed[1] = y

        if theta is not None:
            speed[2] = theta

        with self._lock:
            self._current = speed

        self._motion.post.moveToward(speed[0],
                                     speed[1],
                                     speed[2],
                                     [])

    def stop(self):
        self.set(0,0,0)

