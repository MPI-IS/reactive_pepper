from sensor_msgs.msg import JointState, Range
import tf_conversions
import tf2_ros
import rospy
import threading,time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class Tf_and_odometry_publisher(object):

    
    def __init__(self,
                 frequency,
                 odometry_topic,
                 ros_base_frame,
                 ros_odometry_frame,
                 torso_transform): 

        self._frequency = frequency
        
        self._ros_base_frame = ros_base_frame
        self._ros_odometry_frame = ros_odometry_frame

        self._previous_time_torso = None            

        self._torso_odometry_publisher = rospy.Publisher(odometry_topic, Odometry, queue_size=1)
        self._transform_publisher = tf2_ros.TransformBroadcaster()

        self._torso_transform = torso_transform
        
        self._running = False
        self._thread = None
        
        
    def _publish(self,odometry_data,time_stamp):


        torso_odometry = Odometry()
        
        torso_odometry.pose.pose.position.x = odometry_data[0]
        torso_odometry.pose.pose.position.y = odometry_data[1]
        torso_odometry.pose.pose.position.z = odometry_data[2]

        q = tf_conversions.transformations.quaternion_from_euler(odometry_data[3],
                                                  odometry_data[4],
                                                  odometry_data[5])

        torso_odometry.pose.pose.orientation.x = q[0]
        torso_odometry.pose.pose.orientation.y = q[1]
        torso_odometry.pose.pose.orientation.z = q[2]
        torso_odometry.pose.pose.orientation.w = q[3]

        t = torso_odometry.pose.pose.position
        q = torso_odometry.pose.pose.orientation

        torso_odometry.header.stamp = rospy.Time(time_stamp)

        
        pub_t = TransformStamped()
        pub_t.header.stamp = rospy.Time(time_stamp)
        pub_t.transform.translation = torso_odometry.pose.pose.position
        pub_t.transform.rotation = torso_odometry.pose.pose.orientation
        pub_t.header.frame_id = self._ros_odometry_frame
        pub_t.child_frame_id = self._ros_base_frame

        self._transform_publisher.sendTransform(pub_t)
        self._torso_odometry_publisher.publish(torso_odometry)

        
        
    def _run(self):

        t_sleep = 1.0/self._frequency

        self._running = True

        while self._running:

            d = self._torso_transform.get()
            slept = False
            
            if d:
                values, time_stamp = d
                if time_stamp != self._previous_time_torso:
                    self._previous_time_torso = time_stamp
                    self._publish(values,time_stamp)

                    time.sleep(t_sleep)
                    slept = True

            if not slept:
                time.sleep(t_sleep/10.0)
                
    def start(self):
        if self._thread:
            self.stop()
        self._thread = threading.Thread(target=self._run) 
        self._thread.setDaemon(True)
        self._thread.start()

        
    def stop(self):
        if self._thread :
            self._running=False
            self._thread.join()
            self._thread = None
            
    def __del__(self):
        self.stop()

        
