from sensor_msgs.msg import JointState, Range
from nav_msgs.msg import Odometry
import rospy
import time,threading


class Joint_states_publisher(object):
    
    def __init__(self,
                 frequency,
                 motion_proxy,
                 joints_instance,
                 joint_state_topic):

        self._frequency = frequency
        self._motion_proxy = motion_proxy
        self._previous_time_joints = None
        self._joints_instance = joints_instance
        
        self._joint_state_publisher = rospy.Publisher(joint_state_topic,
                                                      JointState,
                                                      queue_size=1)

        self._thread = None
        self._running = False

        
    def _publish_joint_state(self,joint_values,time_stamp):

        jointState = JointState()
        joints_ = joint_values.keys()
        jointState.name = joint_values.keys()
        jointState.position = [joint_values[joint] for joint in jointState.name]
        jointState.header.stamp = rospy.Time(time_stamp)
        try :
            self._joint_state_publisher.publish(jointState)
        except :
            pass
        
    def _run(self):

        t_sleep = 1.0 / self._frequency
        
        self._running = True

        while self._running:
        
            d = self._joints_instance.get()
            
            slept = False
            
            if d :

                joint_values,time_stamp = d

                if time_stamp != self._previous_time_joints:
                    self._previous_time_joints = time_stamp
                    self._publish_joint_state(joint_values,
                                              time_stamp)
                    time.sleep(t_sleep)
                    slept=True

            if not slept:
                time.sleep(t_sleep/10.0)


    def start(self):
        if self._thread:
            self.stop()
        self._thread = threading.Thread(target=self._run) 
        self._thread.setDaemon(True)
        self._thread.start()

        
    def stop(self):
        if self.thread :
            self._running=False
            self._thread.join()
            self._thread = None

            
    def __del__(self):
        self.stop()

                
