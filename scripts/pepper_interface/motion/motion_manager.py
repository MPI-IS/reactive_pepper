import math,time
from .velocity_to_duration import velocity_to_duration


class Motion_manager:

    SET_ANGLE = 1
    ANGLE_INTERPOLATION = 2
    
    def __init__(self,
                 method, # set_angle or angle_interpolation
                 motion_proxy,
                 target_values,
                 current_values,
                 velocity=None,
                 duration=None,
                 target_time=None):

        self._method = method
        self._motion_proxy = motion_proxy
        self._last_set_values = None
        
        def _get_duration(velocity,
                          duration,
                          target_time,
                          target_values,
                          current_values):

            if duration is not None:
                return duration

            if target_time is not None:
                return target_time - time.time()

            if velocity is not None and velocity != 0:
                return velocity_to_duration(target_values,
                                            current_values,
                                            velocity)

            return None

        self._last_set_values = target_values
        joints, values = [], []
        for joint, value in target_values.iteritems():
                joints.append(joint)
                values.append(float(value))

        duration = _get_duration(velocity,
                                 duration,
                                 target_time,
                                 target_values,
                                 current_values)

        if self._method == self.SET_ANGLE:
            self._mid = self._motion_proxy.post.setAngles(joints, values, velocity)
        else :
            self._mid = self._motion_proxy.post.angleInterpolation(joints,
                                                                   values,
                                                                   duration,
                                                                   True)

    def is_running(self, current_angles = None):
        if not self._mid:
            return False

        if self._method == self.SET_ANGLE:
            current_values = current_angles
            diff = 0.0
        
            for a,b in current_values.iteritems():
                for p,q in self._last_set_values.iteritems():
                    if a == p:
                        if (math.fabs(b-q)>diff):
                            diff = math.fabs(b-q)
            if diff > 0.1:
                return True
            else:
                return False

        if self._method == self.ANGLE_INTERPOLATION:
            return self._motion_proxy.isRunning(self._mid)
        

    def stop(self):
        
        try:
            self._motion_proxy.stop(self._mid)
        except:
            pass
