from .velocity_to_duration import velocity_to_duration
import time,random


def _get_random_posture(current_posture,
                        root_posture,
                        amplitude,
                        velocity):

    def get_random(root,amplitude):
        return random.uniform(root-amplitude/2.0,root+amplitude/2.0)

    target_posture = {joint:get_random(root_posture[joint],amplitude[joint])
                      for joint in root_posture.keys()}
    duration = velocity_to_duration(target_posture,current_posture,velocity)

    return target_posture,duration



class _manager:

    
    def __init__(self,
                 motion,
                 get_joints_function,
                 root_posture,
                 amplitude,
                 velocity):

        self.motion = motion
        self.get_joints_function = get_joints_function
        self.root_posture = root_posture
        self.joints = root_posture.keys()
        self.amplitude = amplitude
        self.velocity = velocity
        self.time_start = None
        self.duration = None
        self.mid = None

        
    def execute(self):
        
        if self.mid is None or not self.motion.isRunning(self.mid):

            current_posture = self.get_joints_function(self.joints)
            
            next_posture,duration = _get_random_posture(current_posture,
                                                        self.root_posture,
                                                        self.amplitude,
                                                        self.velocity)

            joints = next_posture.keys()
            values = [next_posture[joint] for joint in joints]
            
            
            self.mid = self.motion.post.angleInterpolation(joints,
                                                           values,
                                                           duration,
                                                           True)

    
    def stop(self):

        try:
            self.motion.stop(self.mid)
        except:
            pass

        
class Random_motion_manager:

    def __init__(self,
                 motion,
                 get_joints_function,
                 root_posture,
                 amplitude,
                 velocity,synchro=True):

        if not synchro:
            self.managers = {joint:_manager(motion,
                                            get_joints_function,
                                            {joint:root_posture[joint]},
                                            amplitude,
                                            velocity)
                             for joint in root_posture.keys()}
        else : 
            self.managers = {1:_manager(motion,
                                        get_joints_function,
                                        root_posture,
                                        amplitude,
                                        velocity)}

    def execute(self):
        
        for manager in self.managers.values() : manager.execute()

    def stop(self):

        for manager in self.managers.values() : manager.stop()
