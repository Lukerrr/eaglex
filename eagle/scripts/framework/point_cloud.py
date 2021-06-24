#!/usr/bin/env python3

import rospy

from framework.lidar import CLidar
from helpers.ros_globals import *

from sensor_msgs.msg import PointCloud as LidarPointCloud
from eagle_comm.msg import PointCloud as OutputPointCloud
from geometry_msgs.msg import Vector3
from eagle_comm.msg import GsCmdFloat

################################################################################
## A class for the map point cloud management.
## The cloud is formed while the mission execution and sent
## to the ground control station by the corresponding request.
## Cloud is mapped in format id:point (int32:Vector3).
##
##      int32 idX = x * density
##      int32 idY = y * density
##      int32 idZ = z * density
##      id = idX ^ (idY << 8) ^ (idZ << 16) 
##
## Fields list:
##  __lidar - CLidar object ref to get laser scaner points
##  __cloud - the Vector3 points array
##  __density - cloud density in 1/m^3
##  __cloudPub - point cloud topic publisher
##  __densitySub - density values topic subscriber
##
## Methods list:
##  __onDensityChanged - density values topic callback
##  UpdateCloud - receives lidar points and fills the cloud
##  Reset - returns num of points in the cloud
##  GetSize - returns num of points in the cloud
##  GetDensity - returns current cloud density value
##  PublishCloud - sends the cloud to the ground station
################################################################################
class CPointCloud:
    def __init__(self, lidar : CLidar):
        self.__lidar = lidar
        self.__cloud = dict()
        self.__density = 50.0
        self.__cloudPub = rospy.Publisher("eagle_comm/out/point_cloud", OutputPointCloud, queue_size = 10)
        self.__densitySub = rospy.Subscriber("eagle_comm/in/cmd_density", GsCmdFloat, self.__onDensityChanged)
    
    ## Get lidar points and fill the cloud
    def UpdateCloud(self):
        data : LidarPointCloud = self.__lidar.GetPoints()
        if(data == None):
            return
        for pt in data.points:
            x = pt.x
            y = pt.y
            z = pt.z
            nX = int(x * self.__density)
            nY = int(y * self.__density)
            nZ = int(z * self.__density)
            ptId = nX ^ (nY << 8) ^ (nZ << 16)
            if(not (ptId in self.__cloud)):
                # Add a new point into the map
                self.__cloud[ptId] = Vector3(x, y, z)

    ## Density values topic callback
    def __onDensityChanged(self, density):
        if(density.value >= 0.1):
            self.__density = density.value
            rospy.loginfo("CPointCloud: cloud density updated (%f)", self.__density)
        else:
            rospy.logwarn("CPointCloud: required cloud density is invalid (%f)", density.value)

    ## Clear the cloud
    def Reset(self):
        self.__cloud = dict()

    ## Returns num of points in the cloud
    def GetSize(self):
        return len(self.__cloud)

    ## Returns current cloud density value
    def GetDensity(self):
        return self.__density

    ## Send the cloud to the ground station
    def PublishCloud(self):
        if(self.GetSize() > 0):
            msg = OutputPointCloud()
            for value in self.__cloud.values():
                msg.cloud.append(value)
            self.__cloudPub.publish(msg)
            rospy.loginfo("CPointCloud: published %d points to communicator", self.GetSize())
        else:
            rospy.loginfo("CPointCloud: cloud is empty, nothing was published")
