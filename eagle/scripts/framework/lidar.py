#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan

from helpers.math_utils import *

import numpy as np
import tf

################################################################################
## Implements 2D laser scaner data collecting and conversion to points.
##
## Fields list:
##  __srcFov - real field of view of the lidar (in radians)
##  __hfov - half field of view in radians
##  __laserScan - last known lidar data
##  __tfListener - frame transforms listener (tf module)
##  __lidarScanSub - lidar data topic subscruber
##  __lidarDataPub - calculated points data publisher
##
## Methods list:
##  __onLaserDataUpdate - lidar data topic callback function
##  UpdateData - converts current lidar distances to points
##                                      and publishes into the topic
################################################################################
class CLidar:
    def __init__(self, srcFovDeg, effectiveFovDeg):
        self.__srcFov = Deg2Rad(srcFovDeg)
        self.__hfov = Deg2Rad(effectiveFovDeg) / 2.
        self.__laserScan = None
        self.__tfListener = tf.TransformListener()
        self.__lidarScanSub = rospy.Subscriber("scan", LaserScan, self.__onLaserDataUpdate)
        self.__lidarDataPub = rospy.Publisher("eagle/lidar_data", PointCloud, queue_size = 1)

    ## Lidar data topic callback function
    def __onLaserDataUpdate(self, data : LaserScan):
        self.__laserScan = data
    
    ## Convert lidar distances to points and publish into the topic
    def UpdateData(self):
        if(self.__laserScan == None):
            # No data received yet
            return
        
        lidarData = PointCloud()
        lidarData.header.frame_id = "laser"
        step = self.__laserScan.angle_increment

        angle = (np.pi - self.__srcFov / 2.0)
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

        lidarData.header.stamp = rospy.Time(0)
        
        # Transform points from laser frame to map frame
        try:
            lidarDataMap = self.__tfListener.transformPointCloud("map", lidarData)
            self.__lidarDataPub.publish(lidarDataMap)
            self.__laserScan = None
        except Exception as e:
            rospy.logerr("CLidar::UpdateData TFLookupException: %s", str(e))