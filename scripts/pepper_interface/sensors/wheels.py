from ..data_exchange.data import Data

WHEELS = ["WheelB","WheelFL","WheelFR"]
ATTRS = ["electric_current","speed","command"]
KEYS = {"electric_current":"Device/SubDeviceList/{$wheel}/ElectricCurrent/Sensor/Value",
        "speed":"Device/SubDeviceList/${wheel}/Speed/Actuator/Value",
        "command":"Motion/Command/Velocity/{$wheel}"}

class Wheel:

    def __init__(self,name):
        self.name =  name
        for attr in ATTRS:
            setattr(self,attr,None)

    def __str__(self):
        s = [self.name]
        for attr in ATTRS:
            s.append("\t"+attr+"\t"+str(getattr(self,attr)))
        return "\n".join(s)
            
class WheelsSensor(Data):

    def __init__(self):
        Data.__init__(self)

    @classmethod
    def get_key(cls,wheel,attr):
        template = KEYS[attr]
        updated = template.replace("{$wheel}",wheel)
        return updated
        
    @classmethod
    def get_keys(cls):
        keys = []
        for attr in ATTRS:
            for wheel in WHEELS:
                keys.append(cls.get_key(wheel,attr))
        return keys

    def get(self):

        data = Data.get(self)

        if not data:
            return None

        values,time_stamp = data

        wheels = {wheel:Wheel(wheel) for wheel in WHEELS}

        for attr in ATTRS:
            for wheel in WHEELS:
                key = self.get_key(wheel,attr)
                setattr(wheels[wheel],attr,values[key])
        
        return wheels,time_stamp
