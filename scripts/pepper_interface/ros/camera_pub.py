import time,math,threading

import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState, Range, LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
import tf
from tf import transformations
import numpy as np


IMAGE_FRAME_ID = "CameraDepth_optical_frame"
IMAGE_ENCODING = "16UC1"
IMAGE_HEIGHT = 240
IMAGE_WIDTH = 320
IMAGE_NB_LAYERS = 2

IMAGE_ENCODING_RGB = "bgr8"
IMAGE_NB_LAYERS_RGB = 3
IMAGE_FRAME_ID_RGB = "CameraTop_optical_frame"


class camera_messages_info():
    

    def rgb_image_messages(self):
        img = Image()
        img.header.frame_id = IMAGE_FRAME_ID_RGB
        img.height = IMAGE_HEIGHT
        img.width = IMAGE_WIDTH
        nbLayers = IMAGE_NB_LAYERS_RGB
        img.encoding =  IMAGE_ENCODING_RGB
        img.step = IMAGE_WIDTH * nbLayers #960

        infomsg = CameraInfo()
        infomsg.header = img.header
        infomsg.width = img.width
        infomsg.height = img.height

        infomsg.K = [302.57856920034106, 0.0, 155.91103802286761,
                     0.0, 301.68951432602375, 118.65056477571655, 0.0, 0.0, 1.0]
        infomsg.distortion_model = "plumb_bob"
        infomsg.D = [0.08331764881149892, -0.21915883964759514,
                     0.002379895815123969, -2.4159895315804205e-05, 0.0]
        infomsg.R = [1.00, 0.00, 0.00,
                     0.00, 1.00, 0.00,
                     0.00, 0.00, 1.00]
        infomsg.P = [303.5335693359375, 0.0, 155.32197329854534, 0.0, 0.0,
                     302.8251037597656, 118.51240966677415, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return img, infomsg

    def ir_image_messages(self):
        img = Image()
        img.header.frame_id = IMAGE_FRAME_ID
        img.encoding = IMAGE_ENCODING
        img.height = IMAGE_HEIGHT
        img.width = IMAGE_WIDTH
        nbLayers = IMAGE_NB_LAYERS
        img.step = img.width * nbLayers

        infomsg = CameraInfo()

        infomsg.width = img.width
        infomsg.height = img.height

        infomsg.K = [285.56056908448994, 0.0, 158.16012640847876, 0.0,
                     286.15836781135715, 118.67311317638475, 0.0, 0.0, 1.0]
        infomsg.P = [284.64276123046875, 0.0, 156.15113872053917, 0.0, 0.0,
                     285.79339599609375, 117.48199104133892, 0.0, 0.0, 0.0, 1.0, 0.0] 
        infomsg.distortion_model = "plumb_bob"
        infomsg.D = [-0.02780656653355008, 0.04779544561062526,
                     -0.0030294086088610223, -0.004929903744084704, 0.0]
        infomsg.R = [1.000000, 0.000000, 0.000000,
                     0.000000, 1.000000, 0.000000,
                     0.000000, 0.000000, 1.000000]

        return img, infomsg

  

    def depth_image_messages(self):

        img = Image()
        img.header.frame_id = IMAGE_FRAME_ID
        img.encoding = IMAGE_ENCODING
        img.height = IMAGE_HEIGHT
        img.width = IMAGE_WIDTH
        nbLayers = IMAGE_NB_LAYERS
        img.step = img.width * nbLayers

        infomsg = CameraInfo()

        infomsg.width = img.width
        infomsg.height = img.height

        infomsg.K = [285.56056908448994, 0.0, 158.16012640847876, 0.0,
                     286.15836781135715, 118.67311317638475, 0.0, 0.0, 1.0]
        infomsg.P = [284.64276123046875, 0.0, 156.15113872053917, 0.0, 0.0,
                     285.79339599609375, 117.48199104133892, 0.0, 0.0, 0.0, 1.0, 0.0] 

        infomsg.D = [] # do not fill !
        infomsg.binning_x = 0
        infomsg.binning_y = 0
        infomsg.distortion_model = ""
        return img, infomsg
 
