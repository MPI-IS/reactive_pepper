from .velocity_to_duration import velocity_to_duration


class Repetitive_motion_manager :


    def __init__(self,
                 motion_proxy,
                 get_joints_function,
                 postures,
                 velocity,
                 initial_velocity,
                 nb_batchs=5):

        self._motion_proxy = motion_proxy
        self._get_joints_function = get_joints_function
        self._postures = postures
        self._velocity = velocity
        self._initial_velocity = initial_velocity
        self._nb_batchs = nb_batchs
        self._durations = self._get_durations(postures,velocity)
        self._mid = None

    def _get_durations(self,postures,velocity):

        durations = [None for _ in range(len(postures))]
        for i in range(1,len(postures)):
            durations[i]=velocity_to_duration(postures[i],
                                              postures[i-1],
                                              velocity)
        return durations
        
        
    def _generate_command(self,
                          postures,
                          nb_batchs):

        postures_ = postures*nb_batchs
        joints = postures[0].keys()

        time_lists = []
        values = []

        current_posture = self._get_joints_function(joints)
        initial_duration = velocity_to_duration(postures_[0],
                                                current_posture,
                                                self._initial_velocity)

        time_lists_ = [initial_duration]
        t = initial_duration
        count = 1
        for _ in range(len(postures_)-1):
            t+=self._durations[count]
            time_lists_.append(t)
            count+=1
            if count>=len(self._durations):
                count = 1
            
        for joint in joints:
            time_lists.append(time_lists_)
            values_ = [posture[joint] for posture in postures_]
            values.append(values_)
            
            
        return joints,values,time_lists
        

    def is_running(self):

        if self._mid is None:
            return False

        try :
            if self._motion_proxy.isRunning(self._mid):
                return True
        except Exception as e:
            return False

        return False

    
    def stop(self):

        try :
            self._motion_proxy.stop(self._mid)
            self._mid = None
        except :
            pass
    
        
    def update(self):

        if not self.is_running():

            joints,values,time_lists = self._generate_command(self._postures,
                                                              self._nb_batchs)

            self._mid = self._motion_proxy.post.angleInterpolation(joints,
                                                                   values,
                                                                   time_lists,
                                                                   True)
            

