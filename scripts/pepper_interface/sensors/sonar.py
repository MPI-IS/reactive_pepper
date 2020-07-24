from ..data_exchange.data import Data

_KEY_FRONT = 'Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value'
_KEY_BACK = 'Device/SubDeviceList/Platform/Back/Sonar/Sensor/Value'

class Sonar(Data):

    def __init__(self):
        Data.__init__(self)
    
    @classmethod
    def get_keys(cls):
        return [_KEY_FRONT,_KEY_BACK]

    def get(self):

        data = Data.get(self)

        if not data:
            return None

        values,time_stamp = data

        for key,value in values.iteritems():
            if "Front" in key :
                front = value
            else :
                back = value

        r = {}
        r["front"]=front
        r["back"]=back

        return r,time_stamp
