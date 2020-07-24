import time,math,threading

import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState, Range, LaserScan, PointCloud2, PointField
import numpy as np

LASER_FRONT_FRAME_ID = 'SurroundingFrontLaser_frame'
LASER_LEFT_FRAME_ID = 'SurroundingLeftLaser_frame'
LASER_RIGHT_FRAME_ID = 'SurroundingRightLaser_frame'

LASER_MIN_ANGLE = -0.523598776
LASER_MAX_ANGLE = 0.523598776
LASER_MAX_RANGE = 5.0
LASER_MIN_RANGE = 0.1
LASER_FOV = math.fabs(LASER_MIN_ANGLE) + math.fabs(LASER_MAX_ANGLE)
LASER_SCAN_NUM = 15   

class laser_messages_info:
    
    def laser_messages(self):

        laser_messages = [LaserScan(), LaserScan(), LaserScan()]

        laser_messages[0].header.frame_id = LASER_FRONT_FRAME_ID
        laser_messages[1].header.frame_id = LASER_LEFT_FRAME_ID
        laser_messages[2].header.frame_id = LASER_RIGHT_FRAME_ID

        inx = 0
        for i in laser_messages:
            laser_messages[inx].angle_min = LASER_MIN_ANGLE
            laser_messages[inx].angle_max = LASER_MAX_ANGLE
            laser_messages[inx].angle_increment = LASER_FOV / LASER_SCAN_NUM
            laser_messages[inx].range_min = LASER_MIN_RANGE
            laser_messages[inx].range_max = LASER_MAX_RANGE
            inx += 1

        return laser_messages

    def xyz_array_to_pointcloud2(self,points, stamp, frame_id):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        '''
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*len(points)
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()

        return msg 
