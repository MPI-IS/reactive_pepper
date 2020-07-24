import threading,copy


class Data(object) :

    def __init__(self,data_transform=None):
        self._data = None
        self._lock = threading.Lock()
        self._data_transform = data_transform
        data_transform=None
        
    def set(self,data):
        with self._lock:
            self._data = data
            if self._data_transform:
                self._data_transform(self._data)
                    
    def get(self):
        with self._lock:
            r = copy.deepcopy(self._data)
        return r

