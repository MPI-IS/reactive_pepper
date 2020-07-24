import time
import rospy
from configuration import configuration

from naoqi_.proxies import Proxies
from naoqi_.memory import Memory
from naoqi_.torso_transform import Torso_transform

from motion.dance_manager import Dance_manager
from motion.random_motion import Random_motion_manager
from motion.motion_manager import Motion_manager
from motion.repetitive_motion_manager import Repetitive_motion_manager
from motion.feedforward_interpolation import Feedforward_interpolation
from motion.tracker import Tracker

from body.joints import Joints
from body.leds import Leds
from body.head import Head
from body.arms import Arms
from body.battery import Battery
from body.tts import TTS
from body.tablet import Tablet
from body.knee import Knee
from body.hip import Hip
from body.wheels import Wheels
from body.music import Music
from body.hands import Hands

from sensors.camera import Camera

from ros.transforms import Transforms

from ros.tf_and_odometry_publisher import Tf_and_odometry_publisher
from ros.joint_states_publisher import Joint_states_publisher

from ros.publishers import Publishers


class Pepper_interface(object):


    def __init__(self,IP,PORT,
                 simulation,
                 face_detection=False,
                 publish=False,
                 camera_calibration=False,
                 stop=True):

        self.simulation = simulation

        self.stop_ = stop
        stop=True
        
        try :
            rospy.init_node("playful_ros_node_pepper_interface")
        except Exception as e:
            print
            print "attempted to start ROS node but failed "
            print "with error message: ",str(e)
            print "Did you launch your ROS core or initiated another node?"
            print


        if camera_calibration:
            self.use_ir_camera = True
        else :
            self.use_ir_camera = False

        self.transforms = Transforms(configuration.frames.reference,
                                     configuration.frames.relative,
                                     configuration.frames.head,
                                     configuration.frames.camera,
                                     configuration.camera.horizontal_fov)
        
        self._proxies = Proxies(IP,PORT,
                                simulation,
                                face_detection,
                                face_detection_subscription=configuration.face_detection.subscription,
                                face_detection_period=configuration.face_detection.period)

        self._naoqi_memory = Memory(self.transforms,
                                    self._proxies.memory,
                                    configuration.memory.frequency,
                                    configuration.body.joints,
                                    configuration.laser.scan_num,
                                    face_detection,
                                    configuration.frames.relative,
                                    configuration.frames.reference,
                                    configuration.face_detection.detection_id)
                                              
        self.sonar = self._naoqi_memory.sonar
        self.wheels_sensor = self._naoqi_memory.wheels
        self.laser = self._naoqi_memory.laser
        self.head_touch = self._naoqi_memory.head_touch
        self.left_hand_touch = self._naoqi_memory.left_hand_touch
        self.right_hand_touch = self._naoqi_memory.right_hand_touch
        
        self.face_detection = self._naoqi_memory.face_detection
        self.temperatures = self._naoqi_memory.temperatures

        self.joints = Joints(self._proxies.motion,
                             configuration.body.frequency,
                             self.simulation)

        self._torso_transform = Torso_transform(self._proxies.motion,
                                                configuration.frames.naoqi,
                                                configuration.frames.naoqi_reference,
                                                self.simulation,
                                                configuration.body.frequency)
        
        self.battery = Battery(self._proxies.battery)

        self.wheels = Wheels(self._proxies.motion)

        self.tts = TTS(self._proxies.tts)

        # does not work on naoqi 2.4.3 (works on 2.5)
        try :
            self.tablet = Tablet(self._proxies.tablet)
        except :
            pass
            
        self.music = Music(self._proxies.audio)

        self.leds = Leds(self._proxies.leds)

        self.tracker = Tracker(self._proxies.tracker,
                               self._proxies.memory,
                               configuration.tracker.event,
                               configuration.tracker.event_type,
                               configuration.tracker.effector_id,
                               configuration.tracker.head_threshold)
        
        self.head = Head(configuration.head.method,
                         self._proxies.motion,
                         configuration.head.joint_yaw,
                         configuration.head.joint_pitch,
                         configuration.head.z_shift,
                         self._set_joints_values,
                         self.get_joints_values,
                         self.transforms,
                         configuration.head.smoother_time_threshold,
                         configuration.head.smoother_max_acceleration,
                         configuration.head.wheels_compensation_factor,
                         self.wheels.get)

        self.hip = Hip(configuration.hip.method,
                       configuration.hip.joints,
                       self._set_joints_values,
                       self.get_joints_values)

        self.knee = Knee(configuration.knee.method,
                         configuration.knee.joint[0],
                         self._set_joints_values,
                         self.get_joints_values)

        self.arms = Arms(configuration.arms.method,
                         self._proxies.motion,
                         configuration.arms.length,
                         configuration.arms.left_joints,
                         configuration.arms.right_joints,
                         configuration.hip.joints,
                         configuration.knee.joint,
                         configuration.frames.left_shoulder,
                         configuration.frames.right_shoulder,
                         self._set_joints_values,
                         self.get_joints_values,
                         self.transforms,
                         configuration.arms.on_exit_posture)

        self.hands = Hands(configuration.hands.method,
                           self._set_joints_values,
                           self.get_joints_values)
        
        self._ros_tf_and_odom = Tf_and_odometry_publisher(
            configuration.ros.tf_and_odometry_frequency,
            configuration.ros.odometry_topic,
            configuration.frames.base,
            configuration.frames.reference,
            self._torso_transform)

        self._joint_states_publisher = Joint_states_publisher(
            configuration.ros.joint_states_frequency,
            self._proxies.motion,
            self.joints,
            configuration.ros.joint_states_topic)

        if not self.simulation:
            self.camera = Camera(configuration.camera.frequency,
                     self._proxies.camera,
                     configuration.camera.camera_id,
                     configuration.camera.resolution,
                     configuration.camera.source,
                     configuration.camera.color_space,
                     configuration.camera.frame_rate)
            self.camera_rgb = Camera(configuration.camera_rgb.frequency,
                     self._proxies.camera,
                     configuration.camera_rgb.camera_id,
                     configuration.camera_rgb.resolution,
                     configuration.camera_rgb.source,
                     configuration.camera_rgb.color_space,
                     configuration.camera_rgb.frame_rate)

            if self.use_ir_camera:
                self.camera_ir = Camera(configuration.camera_ir.frequency,
                                        self._proxies.camera,
                                        configuration.camera_ir.camera_id,
                                        configuration.camera_ir.resolution,
                                        configuration.camera_ir.source,
                                        configuration.camera_ir.color_space,
                                        configuration.camera_ir.frame_rate)
            
            if publish:
                self._publish = True
                self._publishers = Publishers(self,
                                              configuration.ros.publication_frequency)
            else :
                self._publish = False

        else:
            self._publish = False

            
    def start(self):

        self._naoqi_memory.start()
        self.joints.start()
        self._torso_transform.start()
        self._ros_tf_and_odom.start()
        self._joint_states_publisher.start()
        if not self.simulation:
            self.camera.start()
            self.camera_rgb.start()
            if self.use_ir_camera:
                self.camera_ir.start()
        if self._publish:
            self._publishers.start()
            
        self._proxies.motion.wakeUp()
        self.arms.set_stiffnesses(True,1.0)
        self.arms.set_stiffnesses(False,1.0)
        self.head.set_stiffnesses(1.0)

        
    def stop(self):

        if not self.stop_ :
            return

        self.tracker.stop()
        
        self.wheels.stop()
        self.head.set(0.0,0.0)
        self.hip.set(0.0,0.0)
        self.knee.set(0.0)
        
        if (self.head.running() or
               self.hip.running() or
               self.knee.running()):
            time.sleep(0.5)


        self.arms.relax() 
        while (self.arms.running(True) or
               self.arms.running(False)):
            time.sleep(0.1)
      
        self.head.set_stiffnesses(0.0)

        if self._publish:
            try :
                self._publishers.stop()
            except :
                pass
        try :
            self._naoqi_memory.stop()
        except :
            pass
        try:
            self.joints.stop()
        except :
            pass
        try:
            self._ros_tf_and_odom.stop()
        except :
            pass
        try:
            self._joint_states_publisher.stop()
        except:
            pass

        try:
            self._torso_transform.stop()
        except:
            pass
        if not self.simulation:
            try :
                self.camera.stop()
                self.camera_rgb.stop()
                if self.use_ir_camera:
                    self.camera_ir.stop()
            except :
                pass



        
    def __del__(self):
        self.stop()
            
        
    def wake_up(self):
        self._proxies.motion.wakeUp()


    def get_joints_values(self,joints=None):

        data = self.joints.get()

        if not data:
            return None

        joints_values,time_stamp = data

        if joints is None:
            return joints_values
        
        return {j:v for j,v in joints_values.iteritems()
                if j in joints}

    
    def _set_joints_values(self,
                           method,
                           joint_values,
                           motion_manager,
                           velocity=0.1,
                           target_time=None):


    	current = self.get_joints_values(joint_values.keys())

    	new_motion_manager = Motion_manager(method,
                                            self._proxies.motion,
                                            joint_values,
                                            current,
                                            velocity=velocity,
                                            target_time=target_time)
        
    	try:
    	    motion_manager.stop()
    	except:
    	    pass

    	return new_motion_manager


    def get_random_motion_manager(self,
                                  center_posture,
                                  amplitudes,
                                  velocity):

        return Random_motion_manager(self._proxies.motion,
                                     self.get_joints_values,
                                     center_posture,
                                     amplitudes,
                                     velocity,
                                     synchro=False)
        
    
    def get_dance_manager(self,
                          postures,
                          beat,
                          initial_velocity,
                          batch_size=4):

    	return Dance_manager(self._proxies.motion,
                             postures,
                             beat,
    			     initial_velocity,
    			     batch_size=batch_size)

    
    def get_repetitive_motion_manager(self,
                                      postures,
                                      velocity,
                                      initial_velocity,
                                      batch_size=5) :
        
        return Repetitive_motion_manager(self._proxies.motion,
                                         self.get_joints_values,
                                         postures,
                                         velocity,
                                         initial_velocity,
                                         nb_batchs=batch_size)


    def get_feedforward_interpolation_manager(self,
                                              postures,
                                              starting_velocity,
                                              velocities):
        joints = postures[0].keys()
        current_posture = self.get_joints_values(joints)
        return Feedforward_interpolation(self._proxies.motion,
                                         current_posture,
                                         postures,
                                         starting_velocity,
                                         velocities)

    
    
class get:

    def __init__(self,IP,PORT,
                 simulation,
                 face_detection=False,
                 publish=False,
                 camera_calibration=False,
                 stop=True):
        self._interface = Pepper_interface(IP,PORT,
                                           simulation,
                                           face_detection=face_detection,
                                           publish=publish,
                                           camera_calibration=camera_calibration,
                                           stop=stop)
        stop=True
        
    def __enter__(self):
        self._interface.start()
        return self._interface

    def __exit__(self,type,value,traceback):
        self._interface.stop()
