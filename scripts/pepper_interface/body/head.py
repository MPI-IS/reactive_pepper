import math,time


class Look_at_smoother:

    def __init__(self,
                 forget_time_threshold,
                 authorized_acceleration):

        self.stamp = None
        self.direction = time.time()-forget_time_threshold
        self.speed = 0
        self.forget_time_threshold = forget_time_threshold
        self.authorized_acceleration = authorized_acceleration

    def _reset(self,direction,stamp):

        self.speed = 0
        self.stamp = stamp
        self.direction = direction
        
    def get(self,
            direction,
            desired,
            stamp):

        if direction != self.direction:
            self._reset(direction,stamp)

        if stamp-self.stamp > self.forget_time_threshold:
            self._reset(direction,stamp)
            
        time_diff = stamp-self.stamp

        if time_diff == 0 :
            return self.speed

        velocity_diff = desired-self.speed
        authorized_diff = self.authorized_acceleration*time_diff

        if abs(velocity_diff)<authorized_diff :
            return desired

        if self.speed < desired :
            self.speed += authorized_diff
            return self.speed
        
        self.speed -= authorized_diff
        return self.speed

        
        

class Head:

    def __init__(self,
                 method,
                 motion_proxy,
                 joint_yaw,
                 joint_pitch,
                 z_shift,
                 set_joints_function,
                 get_joints_function,
                 transforms,
                 look_at_smoother_time_threshold,
                 authorized_acceleration,
                 wheels_compensation_factor,
                 wheels_get_method):

        self._motion_proxy = motion_proxy
        self._method = method
        self._motion_manager_yaw = None
        self._motion_manager_pitch = None
        self._z_shift = z_shift
        self._set_joints_function = set_joints_function
        self._get_joints_function = get_joints_function
        self._joint_yaw = joint_yaw
        self._joint_pitch = joint_pitch
        self._joints = [joint_yaw,joint_pitch]
        self._transforms = transforms
        if authorized_acceleration is not None:
            self._look_at_smoother_yaw = Look_at_smoother(look_at_smoother_time_threshold,
                                                          authorized_acceleration)
            self._look_at_smoother_pitch = Look_at_smoother(look_at_smoother_time_threshold,
                                                            authorized_acceleration)
        else:
            self._look_at_smoother_yaw = None
            self._look_at_smoother_pitch = None
        self._wheels_compensation_factor = wheels_compensation_factor
        self._wheels_get_method = wheels_get_method

            
    def set( self,
             yaw,
             pitch,
             velocity_yaw=0.1,
             velocity_pitch=0.1,
             target_time=None,
             max_yaw=(math.pi/2.0),
             max_pitch=(math.pi/2.0),
             ramping_yaw=(math.pi/5.0),
             ramping_pitch=(math.pi/5.0) ):

        current_joints= self._get_joints_function(self._joints)

        if current_joints is None: return

        current_yaw = current_joints[self._joint_yaw]
        current_pitch = current_joints[self._joint_pitch]

        if yaw is None:
            yaw=current_yaw
        if pitch is None:
            pitch=current_pitch
        
        if yaw > max_yaw :
            yaw = max_yaw
        elif yaw<-max_yaw :
            yaw = -max_yaw

        if pitch > max_pitch :
            pitch = max_pitch
        elif pitch < -max_pitch :
            pitch = -max_pitch

        if self._wheels_compensation_factor is not None:
            _,__,wheels_theta = self._wheels_get_method()
            if yaw < current_yaw:
                sign = 1.0
            else :
                sign = -1.0
            if (sign * wheels_theta) > 0:
                velocity_yaw = velocity_yaw + abs(wheels_theta) * self._wheels_compensation_factor
                if velocity_yaw > 1.0 : velocity_yaw = 1.0
            else :
                velocity_yaw = velocity_yaw - abs(wheels_theta) * self._wheels_compensation_factor
                if velocity_yaw < 0.0 : velocity_yaw = 0.0

        d_yaw = abs(current_yaw-yaw)
        d_pitch = abs(current_pitch-pitch)

        speed_yaw = velocity_yaw
        speed_pitch = velocity_pitch

        if d_yaw <ramping_yaw:
            speed_yaw = velocity_yaw * d_yaw / ramping_yaw
        
        if d_pitch <ramping_pitch:
            speed_pitch = velocity_pitch * d_pitch / ramping_pitch

                
        self._motion_manager_yaw = self._set_joints_function(self._method,
                                                             {self._joint_yaw:yaw},
                                                             self._motion_manager_yaw,
                                                             velocity=speed_yaw,
                                                             target_time=target_time)

        self._motion_manager_pitch = self._set_joints_function(self._method,
                                                               {self._joint_pitch:pitch},
                                                               self._motion_manager_pitch,
                                                               velocity=speed_pitch,
                                                               target_time=target_time)

        velocity_yaw=0.1
        velocity_pitch=0.1
        target_time=None
        

    def stop(self):
        try:
            self._motion_manager_yaw.stop()
            self._motion_manager_pitch.stop()
        except:
            pass

        
    def set_stiffnesses(self,value):
        self._motion_proxy.setStiffnesses( [self._joint_yaw,self._joint_pitch] ,
                                           value )


    def running(self):
        yaw = False
        pitch = False
        if self._motion_manager_yaw:
            yaw = self._motion_manager_yaw.is_running(self._get_joints_function(self._joints))
        if self._motion_manager_pitch:
            pitch = self._motion_manager_yaw.is_running(self._get_joints_function(self._joints))
        return (yaw or pitch)


    
    def look_at( self,
                 position,
                 velocity=0.2,
                 max_yaw=(math.pi/2.0),
                 max_pitch=(math.pi/2.0) ):

        if position is None:
            velocity=0.2
            return None,None

        position_in_relative_frame = self._transforms.in_relative_frame(position)

        if not position_in_relative_frame:
            velocity=0.2
            return None,None

        # computing target yaw and pitch

        dxh = position_in_relative_frame[0]
        dyh = position_in_relative_frame[1]
        target_yaw = math.atan2(dyh, dxh)
        
        dxc = position_in_relative_frame[0]
        dzc = position_in_relative_frame[2]-self._z_shift
        target_pitch = - math.atan2(dzc, dxc)

        t = time.time()
        if self._look_at_smoother_yaw is not None:
            velocity_yaw = self._look_at_smoother_yaw.get( (current_yaw>target_yaw),
                                                           velocity,
                                                           t )
        else :
            velocity_yaw = velocity
        if self._look_at_smoother_pitch is not None:
            velocity_pitch = self._look_at_smoother_pitch.get( (current_pitch>target_pitch),
                                                               velocity,
                                                               t )
        else :
            velocity_pitch = velocity
        
        self.set(target_yaw,
                 target_pitch,
                 velocity_yaw=velocity_yaw,
                 velocity_pitch=velocity_pitch,
                 max_yaw=max_yaw,
                 max_pitch=max_pitch)

        velocity=0.2

        return target_yaw,target_pitch
