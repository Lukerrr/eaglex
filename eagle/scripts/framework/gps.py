#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu

from helpers.math_utils import *

class CGpsSystem:
    def __init__(self):
        self.lat = 0.
        self.lon = 0.
        self.heading = 0.

        self.__acc = None
        self.__mag = None

        self.__gpsPosSub = rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, self.__onPositionChanged)
        self.__magSub = rospy.Subscriber("mavros/imu/mag", MagneticField, self.__onMagneticFieldChanged)
        self.__accSub = rospy.Subscriber("mavros/imu/data", Imu, self.__onAccelerationChanged)
    
    def __onPositionChanged(self, data):
        self.lat = data.latitude
        self.lon = data.longitude

    def __onMagneticFieldChanged(self, data):
        mag = data.magnetic_field
        self.__mag = [mag.x, mag.y, mag.z]

        if(self.__acc == None):
            return
        
        roll = np.arctan2(self.__acc[1], self.__acc[2])
        pitch = np.arctan(-self.__acc[0] / (self.__acc[1] * np.sin(roll) + self.__acc[2] * np.cos(roll)))
        yaw = np.arctan2(self.__mag[2] * np.sin(roll) - self.__mag[1] * np.cos(roll), self.__mag[0] * np.cos(pitch) + self.__mag[1] * np.sin(pitch) * np.sin(roll) + self.__mag[2] * np.sin(pitch) * np.cos(roll))
        
        self.heading = Rad2Deg(yaw)

        if(self.heading < 0.):
            self.heading += 360.0

    def __onAccelerationChanged(self, data):
        lacc = data.linear_acceleration
        self.__acc = [lacc.x, lacc.y, lacc.z]
        

