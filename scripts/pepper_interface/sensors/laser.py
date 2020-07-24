import threading
from ..data_exchange.data import Data


class Laser(Data):


    def __init__(self, transforms,
                 scan_num):
        Data.__init__(self)
        self._transforms = transforms
        self._scan_num = scan_num
        _,self._keys_x,self._keys_y = self._generate_keys()
        
        
    def _generate_keys(self):

        keys_x = {}
        keys_y = {}
        keys = []

        prefixes = []

        id_ = 0

        for orientation in ["Front", "Left", "Right"]:
            k = "Device/SubDeviceList/Platform/LaserSensor/" + orientation + "/Horizontal"
            for i in xrange(self._scan_num, 0, -1):
                keyX = k + '/Seg' + '%02d' % (i,) + '/X/Sensor/Value'
                keyY = k + '/Seg' + '%02d' % (i,) + '/Y/Sensor/Value'
                keys.append(keyX)
                keys.append(keyY)
                keys_x[id_] = keyX
                keys_y[id_] = keyY
                id_ += 1

        return keys,keys_x,keys_y

    
    def get_raw(self):

        data = Data.get(self)
        if not data: return 

        data,time_stamp = data

        laser_points = []
	ids = self._keys_x.keys()
	for id_ in ids:
	    x = data[self._keys_x[id_]]
	    y = data[self._keys_y[id_]]
	    laser_points.append((x, y))

        return laser_points,time_stamp
        
    
    def get(self):

        data = Data.get(self)

        if not data: return 
        
        data,time_stamp = data

	laser_points = []

	ids = self._keys_x.keys()
	for id_ in ids:
	    x = data[self._keys_x[id_]]
	    y = data[self._keys_y[id_]]
	    laser_points.append((x, y))
        
        # dividing the ranges into 3 lasers (since frames of 3 are
        # different)
        front_laser_points = laser_points[0:14]
        left_laser_points = laser_points[15:29]
        right_laser_points = laser_points[30:44]

        # transforming the lasers point to reference frame
        points_front = self._transforms.laser_points_transform(front_laser_points,
                                                              'SurroundingFrontLaser_frame')
        points_left = self._transforms.laser_points_transform(left_laser_points,
                                                             'SurroundingLeftLaser_frame')
        points_right = self._transforms.laser_points_transform(right_laser_points,
                                                              'SurroundingRightLaser_frame')

        # putting the points to one set of laser point in reference frame
        points = {}
        if points_left and points_right and points_front:
            points["front"]=points_front
            points["left"]=points_left
            points["right"]=points_right
        else :
            return None

        return points, time_stamp

        
    
    def get_keys(self):

        return self._generate_keys()[0]

