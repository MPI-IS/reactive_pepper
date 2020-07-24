


class Hands :


    def __init__(self,
                 method,
                 set_joints_function,
                 get_joints_function):

        self._method = method
        self._motion_managers = [None,None]
        self._set_joints_function = set_joints_function
        self._get_joints_function = get_joints_function

        
    def set(self,
            left,
            value,
            velocity=0.1,
            target_time=None):
        
        if left:
            joint="LHand"
            index = 0
        else :
            joint = "RHand"
            index = 1
            
        joint_values = {joint:value}

        self._motion_managers[index] = self._set_joints_function(self._method,
                                                                 joint_values,
                                                                 self._motion_managers[index],
                                                                 velocity=velocity,
                                                                 target_time=target_time)
        
            
        
    def running(self,left):

        if left :
            index = 0
        else :
            index = 1

        try :
            return self._motion_managers[index].is_running()
        except :
            return False


    def stop(self):

        for index in [0,1]:
            try :
                self._motion_managers[index].stop()
            except : pass

        
