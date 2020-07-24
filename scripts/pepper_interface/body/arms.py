import threading,time,math,copy


try :
    from playful_kinematics.playful_kinematics import IK
    _IK_ACTIVATED = True
    _IK = IK("pepper")
except Exception as e:
    _IK_ACTIVATED = False
    _IK_ERROR = str(e)



class _ik_help:

    default_left_set = False
    default_right_set = False
    lock = threading.Lock()
    
    @classmethod
    def set_reference_posture(cls,left,reference):
        if reference is None :
                if left and not cls.default_left_set:
                        cls._set_default_reference(True)
                        cls.default_left_set = True
                        return
                if not left and not cls.default_right_set:
                        cls._set_default_reference(False)
                        cls.default_right_set = True
                        return
                return
        cls._set_reference(left,reference)
        return

    @classmethod
    def _set_reference(cls,left,reference):
        if left:
            cls.default_left_set = False
            _IK.set_reference_posture(left,reference)
            return
        cls.default_right_set = False
        _IK.set_reference_posture(left,reference)

    @classmethod
    def _set_default_reference(cls,side):
        joint_zero = {}
        joints,limits = _IK.get_params(side)
        reference_posture = {}
        zeros = []
        for joint in joints :
            min_,max_ = limits[joint]
            zero = ( min_+max_ ) / 2.0
            reference_posture[joint]=zero
            zeros.append(zero)
        _IK.set_reference_posture(side,reference_posture)



        
class _move_arms_enabled:

    lock = threading.Lock()
    left = True
    right = True

    @classmethod
    def set(cls,left_,right_):
        with cls.lock:
            if left_ is not None:
                cls.left = left_
            if right_ is not None:
                cls.right = right_

    @classmethod
    def get(cls):
        with cls.lock:
            return [cls.left,cls.right]
        
    
        
class Arms:

    class symmetry:

        @classmethod
        def _neutral(cls,posture,side=''):
            r = {(side+joint[1:]):value for joint,value in posture.iteritems()}
            side = ''
            return r
            
        @classmethod
        def left_to_right(cls,left_posture):
            opposite = cls._neutral(left_posture,side='R')
            reverse = ["RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw"]
            for r in reverse:
                opposite[r]=-opposite[r]
            return opposite
            
        @classmethod
        def right_to_left(cls,left_posture):
            opposite = cls._neutral(left_posture,side='L')
            reverse = ["LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw"]
            for r in reverse:
                opposite[r]=-opposite[r]
            return opposite

    def __init__(self,
                 method,
                 motion_proxy,
                 arm_length,
                 left_joints,
                 right_joints,
                 hip_joints,
                 knee_joint,
                 left_shoulder_frame,
                 right_shoulder_frame,
                 set_joints_function,
                 get_joints_function,
                 transforms,
                 on_exit_posture):

        self._method = method
        self._motion_proxy = motion_proxy
        self._arm_length = arm_length
        self._left_joints = left_joints
        self._right_joints = right_joints
        self._ik_blocked_joints = hip_joints+knee_joint
        self._transforms = transforms
        self._left_shoulder_frame = left_shoulder_frame
        self._right_shoulder_frame = right_shoulder_frame
        self._motion_managers = [None,None]
        self._on_exit_posture = on_exit_posture
        self._set_joints_function = set_joints_function
        self._get_joints_function = get_joints_function
        
        
    def get_joints(self,left):
        if left:
            return copy.deepcopy(self._left_joints)
        else:
            return copy.deepcopy(self._right_joints)
        
    def ik(self,
           left,
           xyz,
           hand_orientation,
           reference_posture=None):

    	if not _IK_ACTIVATED:
    	    raise Exception("failed to call IK, as import of playful_kinematics failed with error: "+_IK_ERROR)

    	blocked_joints = self._get_joints_function(self._ik_blocked_joints)

    	with _ik_help.lock:
    	    _ik_help.set_reference_posture(left,reference_posture)
    	    _,__,posture = _IK.get_posture(left,
                                           xyz,
    					   hand_orientation,
    					   blocked_joints=blocked_joints)

    	return {joint:value for joint,value in posture.iteritems()
    		if joint not in blocked_joints.keys()}
			

    def pointing(self,
                 left,
                 point,
		 hand_orientation=[None,None,None],
		 reference_posture=None):

    	if left:
            shoulder_frame = self._left_shoulder_frame
    	else :
            shoulder_frame = self._right_shoulder_frame
    			
        shoulder = self._transforms.get_frame_origin_position(shoulder_frame)

        if not shoulder: return None

        v = [p-s for p,s in zip(point,shoulder)]
        norm = math.sqrt(sum([a**2 for a in v]))
        scaled = [self._arm_length*a/norm for a in v]
        in_reference_frame = [s_+s for s_,s in zip(scaled,shoulder)]

        in_relative_frame = self._transforms.in_relative_frame(in_reference_frame)

        return self.ik(left,in_relative_frame,
                       hand_orientation=hand_orientation,
                       reference_posture=reference_posture)
    			
    		
    def set(self,
            left,
            joint_values,
            velocity=0.1,
            target_time=None):

    	if left:
    	    motion_manager = self._motion_managers[0]
    	    index = 0
    	else:
    	    motion_manager = self._motion_managers[1]
    	    index = 1

    	self._motion_managers[index] = self._set_joints_function(self._method,
                                                                 joint_values,
                                                                 motion_manager,
    							         velocity=velocity,
                                                                 target_time=target_time)

        
    def are_move_arms_enabled(self):
	   return self._motion_proxy.getMoveArmsEnabled("Arms")

    
    def set_move_arms_motion(self,
                             left_enable,
                             right_enable):

        # so that left_enable and right_enable can be None,
        # if user want to remain untouched
        _move_arms_enabled.set(left_enable,right_enable)
        status = _move_arms_enabled.get()
        
    	self._motion_proxy.setMoveArmsEnabled(*status)


    def running(self, left):

    	if left:
    	    index = 0
    	else:
    	    index = 1
                
    	try:
    	    return self._motion_managers[index].is_running()
    	except:
    	    return False

        
    def stop(self,left=None):

        if left is None:
            indexes = [0,1]
        elif left:
            indexes = [0]
        else :
            indexes = [1]

        left=None
        
        for index in [0,1]:
	    try:
	        self._motion_managers[index].stop()
	    except:
	        pass

            
    def set_stiffnesses(self,left,value):

        if left :
            joints = "LArm"
        else :
            joints = "RArm"

        self._motion_proxy.setStiffnesses(joints,value)

        
    def relax(self):

        self.stop()
        
    	joint_values = self._on_exit_posture

    	motion_manager = None
    	velocity = 0.5
    	motion_manager = self._set_joints_function(self._method,
                                                   joint_values,
                                                   motion_manager,
                                                   velocity=velocity)
            
    	while motion_manager.is_running():
            time.sleep(0.1)   

        self.set_stiffnesses(True,0.0)
        self.set_stiffnesses(False,0.0)
