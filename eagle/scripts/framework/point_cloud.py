#!/usr/bin/env python3

import rospy

from eagle_comm.msg import PointCloud
from geometry_msgs.msg import Vector3

from copy import deepcopy

################################################################################
## A class for the map point cloud management.
## The cloud is formed while the mission execution (TODO) and sent
## to the ground control station by the corresponding request
##
## Fields list:
##  __cloud - the Vector3 points array
##  __movCtrl - CMovementController instance ref to get current drone pose
##  __cloudPub - point cloud topic publisher
##
##
## Methods list:
##  GetSize - returns num of points in the cloud
##  PublishCloud - sends the cloud to the ground station
################################################################################
class CPointCloud:
    def __init__(self, movCtrl):
        self.__movCtrl = movCtrl
        self.__cloud = []

        i = 0
        while i < 10000:
            self.__cloud.append(Vector3(i,-i,i))
            i += 1

        self.__cloudPub = rospy.Publisher("eagle_comm/out/point_cloud", PointCloud, queue_size = 1)

    ## Returns num of points in the cloud
    def GetSize(self):
        return len(self.__cloud)

    ## Send the cloud to the ground station
    def PublishCloud(self):
        if(self.GetSize() > 0):
            msg = PointCloud()
            msg.cloud = deepcopy(self.__cloud)
            self.__cloudPub.publish(msg)
            rospy.loginfo("CPointCloud: published %d points to communicator", self.GetSize())
        else:
            rospy.loginfo("CPointCloud: cloud is empty, nothing was published")
