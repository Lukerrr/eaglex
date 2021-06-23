#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan

from helpers.math_utils import *

import numpy as np
import math
import tf

################################################################################
## Implements 2D laser scaner data collecting and conversion to points.
##
## Fields list:
##  __srcFov - real field of view of the lidar (in radians)
##  __hfov - half field of view in radians
##  __clasterSize - lidar distances iteration step size (>= 1)
##  __laserScan - last known lidar data
##  __tfListener - frame transforms listener (tf module)
##  __lidarScanSub - lidar data topic subscruber
##
## Methods list:
##  __onLaserDataUpdate - lidar data topic callback function
##  GetPoints - converts lidar distances to point cloud
################################################################################
class CLidar:
    def __init__(self, srcFovDeg, effectiveFovDeg, clasterSize):
        self.__srcFov = Deg2Rad(srcFovDeg)
        self.__hfov = Deg2Rad(effectiveFovDeg) / 2.
        self.__clasterSize = clasterSize
        self.__laserScan = None
        self.__tfListener = tf.TransformListener()

        if(self.__clasterSize < 1):
            self.__clasterSize = 1

        self.__lidarDataPub = rospy.Publisher("eagle/lidar_data", PointCloud, queue_size = 10) # RVIZ debug
        self.__lidarScanSub = rospy.Subscriber("scan", LaserScan, self.__onLaserDataUpdate)

    ## Lidar data topic callback function
    def __onLaserDataUpdate(self, data : LaserScan):
        self.__laserScan = data
    
    ## Convert lidar distances to points, returns a point cloud
    def GetPoints(self):
        if(self.__laserScan == None):
            # No data received yet
            return None
        
        lidarData = PointCloud()
        lidarData.header.frame_id = "laser"
        step = self.__laserScan.angle_increment

        angle = (np.pi - self.__srcFov / 2.0)
        laserVec = Vec3(0., 0., 1.)
        laserVecFwd = Vec3(-1., 0., 0.)
        laserVec = laserVec.RotateAxis(laserVecFwd, angle)
        i = 0
        while i < len(self.__laserScan.ranges):
            dist = self.__laserScan.ranges[i]
            fwdAngle = abs(np.pi - angle)
            if(fwdAngle <= self.__hfov and not math.isnan(dist) and not math.isinf(dist)):
                pt = laserVec * dist
                lidarData.points.append(Point32(pt.x, pt.y, pt.z))
            laserVec = laserVec.RotateAxis(laserVecFwd, step)
            angle += step
            i += self.__clasterSize

        lidarData.header.stamp = rospy.Time(0)
        
        # Transform points from laser frame to map frame
        outPoints = None
        try:
            outPoints = self.__tfListener.transformPointCloud("map", lidarData)
            self.__lidarDataPub.publish(outPoints) # RVIZ debug
            self.__laserScan = None
        except Exception as e:
            rospy.logerr("CLidar::UpdateData TFLookupException: %s", str(e))
        
        return outPoints