#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan

from helpers.math_utils import *
from helpers.ros_globals import *

import numpy as np
import tf

class CLidar:
    def __init__(self, fieldOfView):
        self.__hfov = Deg2Rad(fieldOfView) / 2.
        self.__laserScan = None
        self.__lidarScanSub = rospy.Subscriber("scan", LaserScan, self.__onLaserDataUpdate)
        self.__lidarDataPub = rospy.Publisher("eagle/lidar_data", PointCloud, queue_size = 1)
    
    def UpdateData(self):
        if(self.__laserScan == None):
            return
        
        lidarData = PointCloud()
        lidarData.header.frame_id = "/laser"
        step = self.__laserScan.angle_increment
        angle = Deg2Rad(45)
        laserVec = Vec3(0., 0., 1.)
        laserVecFwd = Vec3(0., 1., 0.)
        laserVec = laserVec.RotateAxis(laserVecFwd, angle)
        for dist in self.__laserScan.ranges:
            fwdAngle = abs(np.pi - angle)
            if(fwdAngle <= self.__hfov):
                pt = laserVec * dist
                lidarData.points.append(Point32(pt.y, pt.x, pt.z))
            laserVec = laserVec.RotateAxis(laserVecFwd, step)
            angle += step

        lidarData.header.stamp = now()
        
        # Transform points from laser frame to map frame
        try:
            tl = tf.TransformListener()
            lidarDataMap = tl.transformPointCloud("/map", lidarData)
            self.__lidarDataPub.publish(lidarDataMap)
            self.__laserScan = None
        except Exception as e:
            rospy.logerr("CLidar::UpdateData TFLookupException: %s", str(e))
        
    def __onLaserDataUpdate(self, data : LaserScan):
        self.__laserScan = data