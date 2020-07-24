import qi,time


class Tracker :


    def __init__(self,
                 tracker_proxy,
                 memory_proxy,
                 event,
                 event_type,
                 effector_id,
                 head_threshold):

        self._tracker_proxy = tracker_proxy
        self._memory_proxy = memory_proxy
        self._event = event
        self._event_type = event_type
        self._effector_id = effector_id
        self._head_threshold = head_threshold
        self._running = False
        
    def start(self,mode="Head",velocity=1.0,acceleration=10.0):

        self._tracker_proxy.setMode(mode)
        self._tracker_proxy.setMaximumVelocity(velocity)
        self._tracker_proxy.setMaximumAcceleration(acceleration)
        self._tracker_proxy.trackEvent(self._event)
        self._running = True

        
    def stop(self):

        if self._running:
            self._running=False
            self._tracker_proxy.stopTracker()
            self._tracker_proxy.unregisterAllTargets()


    def set(self,position,time_stamp=None):

        if len(position)==3:
            position = position+[0.0,0.0,0.0]
        elif len(position)==2:
            position = position+[0.0,0.0,0.0,0.0]

        if time_stamp is None:
            time_stamp = qi.clockNow()
        
        al_value = [position,
                    [time.time(),time_stamp],
                    self._effector_id]

        if self._head_threshold is not None:
            al_value.append(self._head_threshold)
            
        if self._event_type == "event":
            self._memory_proxy.raiseEvent(self._event,al_value)
        else :
            self._memory_proxy.raiseMicroEvent(self._event,al_value)
