import math
import tf2_ros
import tf
import tf2_geometry_msgs
import rospy
from geometry_msgs.msg import PoseStamped


class Transforms(object):

    
    def __init__(self,
                 reference_frame,
                 relative_frame,
                 head_frame,
                 camera_frame,
                 horizontal_fov):

        self._tf2buffer = tf2_ros.Buffer()
        self._transform_listener = tf2_ros.TransformListener(self._tf2buffer)

        self._reference_frame = reference_frame
        self._relative_frame = relative_frame
        self._head_frame = head_frame
        self._camera_frame = camera_frame
        self._horizontal_fov = horizontal_fov
        

    def get_reference_frame(self):
        return self._reference_frame

    def get_relative_frame(self):
        return self._relative_frame
        
    def to_from(self,
                positions,
                frame_transform_to,
                frame_id=None,
                orientation=False,
                time_stamp=None):

        frame_id_ = frame_id
        orientation_ = orientation
        frame_id = None
        orientation = False
        
        if frame_id_ is None:
            frame_id_ = self._reference_frame

        if len(positions)==0 : return []
        try :
            p = float(positions[0])
            positions = [positions]
            is_list = False
        except :
            is_list = True

        r = []

        for position in positions:

            if position is None:

                if orientation_ is False:
                    r.append(None)
                else :
                    r.append((None,None))

            else :
                    
                point = PoseStamped()
                point.pose.position.x = position[0]
                point.pose.position.y = position[1]
                try :
                        point.pose.position.z = position[2]
                except :
                        point.pose.position.z = 0.0
                point.header.frame_id = frame_id_
                point.header.stamp = rospy.Time(0)

                if time_stamp is None:
                    stamp = rospy.Time(0)
                else :
                    stamp = time_stamp
                
                try:
                        trans = self._tf2buffer.lookup_transform(frame_transform_to,
                                                                 frame_id_,
                                                                 stamp,
                                                                 rospy.Duration(.1))
                except:
                        return None

                robot_frame_position = tf2_geometry_msgs.do_transform_pose(point,
                                                                           trans)

                if orientation_ is False:

                    r.append ( [robot_frame_position.pose.position.x,
                                    robot_frame_position.pose.position.y,
                                    robot_frame_position.pose.position.z] )

                else:

                    quaternion = (robot_frame_position.pose.orientation.x,
                                      robot_frame_position.pose.orientation.y,
                                      robot_frame_position.pose.orientation.z,
                                      robot_frame_position.pose.orientation.w)

                    euler = tf.transformations.euler_from_quaternion(quaternion)

                    r.append( [robot_frame_position.pose.position.x,
                               robot_frame_position.pose.position.y,
                               robot_frame_position.pose.position.z], euler )

        if is_list :
            return r

        return r[0]


            

    def laser_points_transform(self, points, frame_id):

        try:
            trans = self._tf2buffer.lookup_transform(self._reference_frame,
                                                     frame_id,
                                                     rospy.Time(0),
                                                     rospy.Duration(.1))
        except:
            return None

        transformed_points = []

        for point in points:
                pose_stamped_point = PoseStamped()
                pose_stamped_point.pose.position.x = point[0]
                pose_stamped_point.pose.position.y = point[1]
                pose_stamped_point.pose.position.z = 0
                pose_stamped_point.header.frame_id = frame_id
                pose_stamped_point.header.stamp = rospy.Time()

                transformed_point = tf2_geometry_msgs.do_transform_pose(pose_stamped_point,
                                                                        trans)

                transformed_points.append([transformed_point.pose.position.x,
                                           transformed_point.pose.position.y,
                                           transformed_point.pose.position.z])

        return transformed_points



    def in_fov_2d(self, point):

        if point is None:
            return None
        
        in_head_frame = self.in_head_frame(point)

        if in_head_frame is None:
            return None

        angle = math.atan2(in_head_frame[1],in_head_frame[0])

        if abs(angle) < ( self._horizontal_fov / 2.0 ) :
            return True

        return False
        

    def get_frame_origin_position(self,frame_id):
        return self.to_from([0,0,0],self._reference_frame,frame_id=frame_id)
	

    def in_relative_frame(self,position):
        return self.to_from(position, self._relative_frame, frame_id=self._reference_frame)


    def in_reference_frame(self,position):
        return self.to_from(position, self._reference_frame, frame_id=self._relative_frame)


    def in_camera_frame(self,position):
        return self.to_from(position, self._camera_frame, frame_id=self._reference_frame)


    def in_head_frame(self,position):
        return self.to_from(position, self._head_frame, frame_id=self._reference_frame)

    
    def robot_position(self):
        return self.in_reference_frame([0,0,0])
	
    def robot_orientation(self):
        p = self.robot_position()
        d = self.in_reference_frame([1,0,0])
        if p is None or d is None : return None
        v = [d_ - p_ for p_,d_ in zip(p[:2],d[:2])]
        return math.atan2(v[1],v[0])
