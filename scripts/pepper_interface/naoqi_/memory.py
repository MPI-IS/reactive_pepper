import threading,time
from ..sensors.sonar import Sonar
from ..sensors.wheels import WheelsSensor
from ..sensors.temperatures import Temperatures
from ..sensors.laser import Laser
from ..sensors.head_touch import Head_touch
from ..sensors.hand_touch import Left_hand_touch
from ..sensors.hand_touch import Right_hand_touch
from ..sensors.face_detection import Face_detection
from ..data_exchange.data_extractor import Data_extractor


class _Query_naoqi_memory(object):

    def __init__(self,
                 memory_proxy,
                 instances):

        self._memory_proxy = memory_proxy
        self._keys_dict = {instance.__class__.__name__:instance.get_keys()
                           for instance in instances}
        self._keys = []
        [self._keys.extend(v) for v in self._keys_dict.values()]

    def __call__(self):

        data = self._memory_proxy.getListData(self._keys)
        data = {k: d for k, d in zip(self._keys, data)}
        return data


class _Dispatched_queried_data(object):


    def __init__(self,instances):

        self._instances_keys = [(instance,instance.get_keys())
                                for instance in instances]


    def __call__(self,data):

        current_time = time.time()
        
        for instance,keys in self._instances_keys:
            d = {k:v for k,v in data.iteritems()
                 if k in keys}
            instance.set([d,current_time])


class Memory(Data_extractor):

    def __init__(self,
                 transforms,
                 memory_proxy,
                 frequency,
                 joints,
                 laser_scan_num,
                 face_detection=False,
                 face_detection_frame=None,
                 reference_frame=None,
                 face_detection_id=None):

        self._memory_proxy = memory_proxy

        self.sonar = Sonar()
        self.wheels = WheelsSensor()
        self.temperatures = Temperatures(joints)
        self.laser = Laser(transforms,laser_scan_num)
        self.head_touch = Head_touch()
        self.left_hand_touch = Left_hand_touch()
        self.right_hand_touch = Right_hand_touch()
        
        instances = [self.sonar,self.wheels,
                     self.temperatures,
                     self.laser,self.head_touch,
                     self.left_hand_touch,
                     self.right_hand_touch]

        self.face_detection = None
        if(face_detection):
            self.face_detection = Face_detection(transforms,
                                                 fame_detection_frame,
                                                 reference_frame,
                                                 face_detection_id)
            instances.append(self.face_detection)

        query = _Query_naoqi_memory(memory_proxy,instances)
        dispatch = _Dispatched_queried_data(instances)

        Data_extractor.__init__(self,
                                frequency,
                                query,
                                dispatch)
        
