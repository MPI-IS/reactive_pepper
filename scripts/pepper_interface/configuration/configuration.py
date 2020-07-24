import math


class method:

    set_angle = 1
    angle_interpolation = 2

    
class head:

    joint_yaw = "HeadYaw"
    joint_pitch = "HeadPitch"
    joints = [joint_yaw,joint_pitch]
    method = method.set_angle
    #method = method.angle_interpolation # !!! not adviced, head becomes instable if used same time as wheels !!!
    smoother_time_threshold = 0.3
    #smoother_max_acceleration = 0.55
    smoother_max_acceleration = None
    z_shift=1.05
    wheels_compensation_factor=0.1
    
class hip:

    joints = ["HipRoll", "HipPitch"]
    method = method.set_angle

class hands:

    method = method.angle_interpolation
    
class arms:

    length = 0.1812+0.15+0.03
    left_joints = ['LShoulderPitch',
                   'LShoulderRoll',
                   'LElbowYaw',
                   'LElbowRoll',
                   'LWristYaw']

    right_joints = ['RShoulderPitch',
                    'RShoulderRoll',
                    'RElbowYaw',
                    'RElbowRoll',
                    'RWristYaw']

    hand_joints = ["LHand", "RHand"]

    joints = right_joints+left_joints

    on_exit_posture = { 'RElbowRoll': 0.0,
                        'RElbowYaw': 0.5,
                        'RShoulderRoll': 0.0,
                        'RWristYaw': 0.8,
                        'RShoulderPitch': 1.13,
                        'LShoulderPitch': 1.12,
                        'LShoulderRoll': 0.0,
                        'LElbowYaw': -0.5,
                        'LElbowRoll': 0.0,
                        'LWristYaw': -0.8 } 

    method = method.angle_interpolation
    
    
class knee:

    joint = ["KneePitch"]
    method = method.set_angle

    
class body :

    joints = head.joints + hip.joints + knee.joint + arms.left_joints + arms.right_joints
    frequency = 40
    
    
class laser:

    scan_num = 15
    
class memory:

    frequency = 20

class motion:

    frequency = 40

    
class tracker:

    event = "ALTraker/playful"
    effector_id = 0 # middle of the eyes
    head_threshold=  None
    event_type = "event"
    #event_type = "micro_event"
    
class camera:

    frequency = 30
    camera_id = "pepper_interface_depth_camera"
    resolution = 1
    source = 2
    color_space = 17
    frame_rate = 30
    horizontal_fov = 72.0*0.0174533
    vertical_fov = 58.0*0.0174533
    
class camera_rgb:

    frequency = 30
    camera_id = "pepper_interface_rgb_camera"
    source = 0
    color_space = 13
    resolution = 1 #320x240
    frame_rate = 30

class camera_ir:
    
    frequency = 30
    camera_id = "pepper_interface_depth_ir"
    source = 2
    color_space = 20
    resolution = 1 #320x240
    frame_rate = 30

class frames:

    reference = "odom"
    relative = "base_footprint"
    base = "base_link"
    head = "Head"
    camera = "CameraDepth_frame"
    left_shoulder = "LShoulder"
    right_shoulder = "RShoulder"
    naoqi_reference = 1 # WORLD
    naoqi = "Torso"
    
    
class face_detection:

    detection_id = "pepper_interface_face_detected"
    subscription = "pepper_interface_face_detection"
    period = 500

    
class ros :

    joint_states_frequency = 40
    tf_and_odometry_frequency = 40
    odometry_topic = "odom"
    joint_states_topic = "joint_states"
    publication_frequency = 40
