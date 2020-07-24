import time,math,threading
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState, Range, LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from .laser_pub import laser_messages_info 
from .sonar_pub import sonar_messages_info 
from .camera_pub import camera_messages_info

SONAR_FRONT = 'pepper_interface/sonar/front'
SONAR_BACK = 'pepper_interface/sonar/back'
LASER_SCAN_FRONT = 'pepper_interface/laser/front'
LASER_SCAN_LEFT = 'pepper_interface/laser/left'
LASER_SCAN_RIGHT = 'pepper_interface/laser/right'
DEPTH_CAMERA_TOPIC = 'pepper_interface/depth/image'
CAMERA_INFO_TOPIC = 'pepper_interface/depth/camera_info'
LASER_CLOUD_TOPIC = 'pepper_interface/laser_cloud'
RGB_CAMERA_TOPIC = 'pepper_interface/rgb/image'
RGB_CAMERA_INFO_TOPIC = 'pepper_interface/rgb/camera_info'
IR_CAMERA_TOPIC = 'pepper_interface/ir/image'
IR_CAMERA_INFO_TOPIC = 'pepper_interface/ir/camera_info'

class Publishers(object):

    def __init__(self,
                 pepper_interface,
                 frequency):

        self._thread = None
        self._running = False
        
        self._frequency = frequency

        self._pepper_interface = pepper_interface

        self._previous_time_camera = None
        self._previous_time_sonar = None
        self._previous_time_laser = None
        self._previous_time_laser_cloud = None
        self._previous_time_camera_rgb = None
        self._previous_time_camera_ir = None
        
        self._sonar_publisher_front = rospy.Publisher(SONAR_FRONT,
                                                      Range,
                                                      queue_size=5)
        self._sonar_publisher_back = rospy.Publisher(SONAR_BACK,
                                                     Range,
                                                     queue_size=5)
        
        self._laser_front_publisher = rospy.Publisher(LASER_SCAN_FRONT,
                                                      LaserScan,
                                                      queue_size=1)
        self._laser_left_publisher = rospy.Publisher(LASER_SCAN_LEFT,
                                                     LaserScan,
                                                     queue_size=1)
        self._laser_right_publisher = rospy.Publisher(LASER_SCAN_RIGHT,
                                                      LaserScan,
                                                      queue_size=1)
        
        self._publisher_img_ = rospy.Publisher(DEPTH_CAMERA_TOPIC,
                                               Image,
                                               queue_size=5)
        self._publisher_info_ = rospy.Publisher(CAMERA_INFO_TOPIC,
                                                CameraInfo,
                                                queue_size=5)
        self._publisher_img_rgb_ = rospy.Publisher(RGB_CAMERA_TOPIC,
                                                   Image,
                                                   queue_size=5)
        self._publisher_info_rgb_ = rospy.Publisher(RGB_CAMERA_INFO_TOPIC,
                                                    CameraInfo,
                                                   queue_size=5)
        if self._pepper_interface.use_ir_camera :
            self._publisher_img_ir_ = rospy.Publisher(IR_CAMERA_TOPIC,
                                                      Image,
                                                      queue_size=5)
            self._publisher_info_ir_ = rospy.Publisher(IR_CAMERA_INFO_TOPIC,
                                                       CameraInfo,
                                                       queue_size=5)
            
        self.laser_cloud_pub = rospy.Publisher(LASER_CLOUD_TOPIC,
                                               PointCloud2,
                                               queue_size=5)
       
    def _run(self):

        time_sleep = 1.0/self._frequency

        cameras_ros = camera_messages_info()
        image,camera_info = cameras_ros.depth_image_messages()
        image_rgb, camera_info_rgb = cameras_ros.rgb_image_messages()

        if self._pepper_interface.use_ir_camera:
            image_ir, camera_info_ir = cameras_ros.ir_image_messages()

        sonars_ros = sonar_messages_info()
        sonars = sonars_ros.sonar_messages()
        lasers_ros = laser_messages_info()
        lasers = lasers_ros.laser_messages()

        self._running = True
        
        while self._running:

            image_data = self._pepper_interface.camera.get()

            if image_data is not None:

                image_data,time_stamp = image_data

                if time_stamp != self._previous_time_camera:
                    camera_info.header = image.header
                    image.header.stamp = camera_info.header.stamp = rospy.Time(time_stamp)
                    image.data = image_data
                    self._publisher_img_.publish(image)
                    self._publisher_info_.publish(camera_info)
                    self._previous_time_camera = time_stamp

            image_rgb_data = self._pepper_interface.camera_rgb.get()
            
            if image_rgb_data is not None:

                image_rgb_data,time_stamp = image_rgb_data

                if time_stamp != self._previous_time_camera_rgb:
                    camera_info_rgb.header = image_rgb.header
                    image_rgb.header.stamp = camera_info_rgb.header.stamp = rospy.Time(time_stamp)
                    image_rgb.data = image_rgb_data
                    self._publisher_img_rgb_.publish(image_rgb)
                    self._publisher_info_rgb_.publish(camera_info_rgb)
                    self._previous_time_camera_rgb = time_stamp

            if self._pepper_interface.use_ir_camera:
                image_ir_data = self._pepper_interface.camera_ir.get()

                if image_ir_data is not None:

                    image_ir_data,time_stamp = image_ir_data

                    if time_stamp != self._previous_time_camera_ir:
                        camera_info_ir.header = image_ir.header
                        image_ir.header.stamp = camera_info_ir.header.stamp = rospy.Time(time_stamp)
                        image_ir.data = image_ir_data
                        self._publisher_img_ir_.publish(image_ir)
                        self._publisher_info_ir_.publish(camera_info_ir)
                        self._previous_time_camera_ir = time_stamp
                    
            sonar_data = self._pepper_interface.sonar.get()

            if sonar_data is not None:

                sonar_data,time_stamp = sonar_data

                if time_stamp !=  self._previous_time_sonar:
                    ts = rospy.Time(time_stamp)
                    sonars[0].header.stamp = ts
                    sonars[1].header.stamp = ts

                    sonars[0].range = sonar_data["front"]
                    sonars[1].range = sonar_data["back"]
                    self._sonar_publisher_front.publish(sonars[0])
                    self._sonar_publisher_back.publish(sonars[1])
                    self._previous_time_sonar = time_stamp

                
            laser_data = self._pepper_interface.laser.get_raw()

            if laser_data is not None:
                
                laser_data,time_stamp = laser_data

                if time_stamp != self._previous_time_laser:

                    ts = rospy.Time(time_stamp)
                    lasers[0].header.stamp = ts
                    lasers[1].header.stamp = ts
                    lasers[2].header.stamp = ts 

                    ranges = []
                    for i in laser_data:
                        distance = math.sqrt(math.pow(i[0], 2.0) + math.pow(i[1], 2.0))
                        ranges.append(distance)

                    lasers[0].ranges = ranges[0:14]
                    lasers[1].ranges = ranges[15:29]
                    lasers[2].ranges = ranges[30:44]

                    self._laser_front_publisher.publish(lasers[0])
                    self._laser_left_publisher.publish(lasers[1])
                    self._laser_right_publisher.publish(lasers[2])
                    self._previous_time_laser = time_stamp

            laser_message_cloud_data = self._pepper_interface.laser.get()

            if laser_message_cloud_data is not None:
                laser_data,time_stamp = laser_message_cloud_data

                if time_stamp != self._previous_time_laser_cloud:
                    laser_points = []
                    for laser_position, laser_values in laser_data.iteritems():
                        for point in laser_values:
                            laser_points.append(point)
         
    
                    msg = lasers_ros.xyz_array_to_pointcloud2(laser_points, rospy.Time(laser_message_cloud_data[1]), '/odom')
                    self.laser_cloud_pub.publish(msg)
                    self.previous_time_laser_cloud = time_stamp

            time.sleep(time_sleep)


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

        
