from ..data_exchange.data import Data


_KEYS = { "front":"Device/SubDeviceList/Head/Touch/Front/Sensor/Value",
          "rear":"Device/SubDeviceList/Head/Touch/Rear/Sensor/Value",
          "middle":"Device/SubDeviceList/Head/Touch/Middle/Sensor/Value" }


class Head_touch(Data):


    def __init__(self):
        Data.__init__(self)

        
    def get_keys(self):

        global _KEYS
        return _KEYS.values()

    
    def get(self):

        global _KEYS
        
        data = Data.get(self)

        if data is None:
            return None

        data,time_stamp = data

        r = {}
        
        for side,memory_key in _KEYS.iteritems():
            r[side]=data[memory_key]

        return r,time_stamp
