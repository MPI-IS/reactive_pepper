import math

class Knee:


    def __init__(self,
                 method,
                 knee_joint,
                 set_joints_function,
                 get_joints_function):

        self._method = method
        self._motion_manager = None
        self._set_joints_function = set_joints_function
        self._get_joints_function = get_joints_function
        self._joint = knee_joint
        
    def set(self,
            knee,
            velocity=0.1,
            target_time=0):

	joint_angles = {self._joint:knee}
        
        self._motion_manager = self._set_joints_function(self._method,
                                                         joint_angles,
                                                         self._motion_manager,
                                                         velocity=velocity,
                                                         target_time=target_time)

    def running(self):
        if self._motion_manager:
            return self._motion_manager.is_running(self._get_joints_function(self._joint))

        
    def stop(self):
        try:
            self._motion_manager.stop()
	except:
	    pass
