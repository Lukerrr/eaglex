#!/usr/bin/env python

import rospy

from eagle_comm.msg import PointCloud
from geometry_msgs.msg import Vector3

from copy import deepcopy

class CPointCloud:
    def __init__(self, movCtrl):
        self.__movCtrl = movCtrl
        #self.__cloud = []
        self.__cloud = [\
            Vector3(0, 0, 0),\
            Vector3(1, 1, 1),\
            Vector3(2, 2, 2),\
            Vector3(3, 3, 3)
        ]

        self.__cloudPub = rospy.Publisher("eagle_comm/out/point_cloud", PointCloud, queue_size = 1)
    
    def GetSize(self):
        return len(self.__cloud)
    
    def PublishCloud(self):
        if(self.GetSize() > 0):
            msg = PointCloud()
            msg.cloud = deepcopy(self.__cloud)
            self.__cloudPub.publish(msg)
            rospy.loginfo("CPointCloud: published %d points to communicator", self.GetSize())
        else:
            rospy.loginfo("CPointCloud: cloud is empty, nothing was published")
