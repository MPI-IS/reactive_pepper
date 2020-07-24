import time,math,threading
import rospy
from sensor_msgs.msg import Image, CameraInfo, JointState, Range, LaserScan, PointCloud2, PointField

SONAR_FRONT_FRAME_ID = 'SonarFront_frame'
SONAR_BACK_FRAME_ID = 'SonarBack_frame'
SONAR_MIN_RANGE = 0.25
SONAR_MAX_RANGE = 2.55
SONAR_FIELD_OF_VIEW = 1.05
SONAR_RADIATION_TYPE = Range.ULTRASOUND

class sonar_messages_info():
    
    def sonar_messages(self):

        sonar_message = [Range(), Range()]
        
        sonar_message[0].header.frame_id = SONAR_FRONT_FRAME_ID
        sonar_message[1].header.frame_id = SONAR_BACK_FRAME_ID

        inx = 0
        for i in sonar_message:
            sonar_message[inx].min_range = SONAR_MIN_RANGE
            sonar_message[inx].max_range = SONAR_MAX_RANGE
            sonar_message[inx].field_of_view = SONAR_FIELD_OF_VIEW
            sonar_message[inx].radiation_type = SONAR_RADIATION_TYPE
            inx += 1
            
        return sonar_message
    

