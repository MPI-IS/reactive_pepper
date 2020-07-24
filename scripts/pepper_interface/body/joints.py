import time
from ..data_exchange.data_extractor import Data_extractor


class Joints(Data_extractor):

    def __init__(self,motion_proxy,frequency,simulation):

        all_joints = motion_proxy.getBodyNames('Body')
        
        def _get_data():
            values = motion_proxy.getAngles(all_joints,not simulation)
            joint_values = {j:v for j,v in zip(all_joints,values)}
            time_stamp = time.time()
            return (joint_values,time_stamp)
            
        Data_extractor.__init__(self,
                                frequency,
                                _get_data);
    
