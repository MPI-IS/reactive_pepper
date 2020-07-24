class Battery:
    
    def __init__(self,battery_proxy):
        self._battery_proxy = battery_proxy
        
    def get(self):
        return self._battery_proxy.getBatteryCharge()
