from ..data_exchange.data_extractor import Data_extractor
import time

class Camera(Data_extractor) :

    def __init__(self,
                 frequency,
                 camera_proxy,
                 camera_id,
                 resolution,
                 source,
                 color_space,
                 frame_rate):

        if camera_proxy is not None:
            list_items = camera_proxy.getSubscribers()
            for item in list_items:
                if camera_id in item:
                    camera_proxy.unsubscribe(item)

            self._cam_id = camera_proxy.subscribeCamera(camera_id,
                                                        source,
                                                        resolution,
                                                        color_space,
                                                        frame_rate)
        
        def _get_data():

            image_values = camera_proxy.getImageRemote(self._cam_id)

            if image_values: 
                return image_values[6],time.time()
            return None
            
        Data_extractor.__init__(self,
                                frequency,
                                _get_data)
