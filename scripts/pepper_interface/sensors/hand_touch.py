from ..data_exchange.data import Data


_KEYS_LEFT = "Device/SubDeviceList/LHand/Touch/Back/Sensor/Value"
_KEYS_RIGHT = "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value"



class Left_hand_touch(Data):


    def __init__(self):
        Data.__init__(self)

        
    def get_keys(self):

        global _KEYS_LEFT
        return [_KEYS_LEFT]

    
    def get(self):

        global _KEYS_LEFT
        
        data = Data.get(self)

        if data is None:
            return None

        data,time_stamp = data

        return data[_KEYS_LEFT],time_stamp


    
class Right_hand_touch(Data):


    def __init__(self):
        Data.__init__(self)

        
    def get_keys(self):

        global _KEYS_RIGHT
        return [_KEYS_RIGHT]

    
    def get(self):

        global _KEYS_RIGHT
        
        data = Data.get(self)

        if data is None:
            return None

        data,time_stamp = data

        return data[_KEYS_RIGHT],time_stamp
