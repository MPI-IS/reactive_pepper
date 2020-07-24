from ..data_exchange.data import Data


class Temperatures(Data):

    def __init__(self,joints):

        self._joints = joints

        def _data_transform(data):
            if not data: return None
            values,time_stamp = data
            treated = {}
            for d,v in values.iteritems():
                d = d[21:]
                index = d.find("/")
                d = d[:index]
                treated[d]=v
            data[0]=treated
            
        Data.__init__(self,data_transform=_data_transform)
        
    def get_keys(self):
        keys = []
        for joint in self._joints:
            k = "Device/SubDeviceList/" + joint + "/Temperature/Sensor/Value"
            keys.append(k)
        return keys
        
