from ..data_exchange.data_extractor import Data_extractor
import threading
import time

class Torso_transform(Data_extractor):

    def __init__(self,
                 motion_proxy,
                 naoqi_body,
                 naoqi_frame,
                 simulation,
                 frequency):

        self._motion_proxy = motion_proxy
        self._naoqi_body = naoqi_body
        self._naoqi_frame = naoqi_frame
        self._simulation = simulation
        self._frequency = frequency
        
        def _get_data():
            position = self._motion_proxy.getPosition(self._naoqi_body,self._naoqi_frame, not self._simulation)
            return (position, time.time())
            
        Data_extractor.__init__(self,
                                self._frequency,
                                _get_data);
