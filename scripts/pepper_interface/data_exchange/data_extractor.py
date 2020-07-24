import threading,time
from data import Data


class Data_extractor(object):

    def __init__(self,
                 frequency,
                 get_data_function,
                 data_transform_function=None):

        self._get_data = get_data_function

        self._data = Data(data_transform=data_transform_function)

        self._thread = None
        self._running = False
        self._frequency = frequency


    def _run(self):

        self._running = True
        time_sleep = 1.0 / float(self._frequency)

        while self._running:
            data = self._get_data()
            self._data.set(data)
            time.sleep(time_sleep)

            
    def start(self):
        if self._thread:
            self.stop()
        self._thread = threading.Thread(target=self._run) 
        self._thread.setDaemon(True)
        self._thread.start()

        
    def get(self):
        return self._data.get()


    def stop(self):
        if self.thread :
            self._running=False
            self._thread.join()
            self._thread = None

    def __del__(self):
        self.stop()
        
        
