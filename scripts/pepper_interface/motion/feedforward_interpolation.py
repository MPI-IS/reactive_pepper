import math,time
from .velocity_to_duration import velocity_to_duration


class Feedforward_interpolation:

    
    def __init__(self,
                 motion,
                 current_posture,
                 postures,
                 starting_velocity,
                 velocities):

        self._motion_proxy = motion

        joints = postures[0].keys()

        initial_duration = velocity_to_duration(current_posture,
                                                postures[0],
                                                starting_velocity)

        times = [initial_duration]
        total_time = initial_duration
        for index in range(1,len(postures)):
            duration = (velocity_to_duration(postures[index-1],
                                             postures[index],
                                             velocities[index-1]))
            total_time += duration
            times.append(total_time)
        times = [times]*len(joints)

        joint_lists = []
        for joint in joints:
            joint_lists.append([posture[joint] for posture in postures])
            
        self._mid = self._motion_proxy.post.angleInterpolation(joints,
                                                               joint_lists,
                                                               times,
                                                               True)


    def is_running(self):

        return self._motion_proxy.isRunning(self._mid)

    
    def stop(self):

        try:
            self._motion_proxy.stop(self._mid)
        except:
            pass

    
