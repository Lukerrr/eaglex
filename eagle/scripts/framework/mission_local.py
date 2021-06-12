#!/usr/bin/env python3

import rospy

from eagle_comm.msg import GsCmdFloat

from helpers.math_utils import *

import numpy as np

class CMission:
    def __init__(self, movCtrl, gps):
        self.isLocked = True
        self.isBusy = False
        self.hash = 0xFFFFFFFF

        self.__gps = gps
        self.__movCtrl = movCtrl

        sp = 5.0
        self.__path =\
        [\
            [sp,0],\
            [sp,sp],\
            [0,sp],\
            [0,0]\
        ]
        self.__curIdx = 0

        self.targetHeight = 0.0
        self.__tolerance = 0.0

        self.__heightSub = rospy.Subscriber("eagle_comm/in/cmd_height", GsCmdFloat, self.__onHeightChanged)
        self.__toleranceSub = rospy.Subscriber("eagle_comm/in/cmd_tolerance", GsCmdFloat, self.__onToleranceChanged)

    def IsValid(self):
        pathValid = len(self.__path) >= 2
        heightValid = self.targetHeight >= 1.5
        toleranceValid = self.__tolerance > 0.0

        if(not pathValid):
            rospy.logwarn("CMissionLocal::IsValid: invalid path ('%s')", str(self.__path))
        
        if(not heightValid):
            rospy.logwarn("CMissionLocal::IsValid: invalid target height (%f)", self.targetHeight)
        
        if(not toleranceValid):
            rospy.logwarn("CMissionLocal::IsValid: invalid tolerance (%f)", self.__tolerance)

        return pathValid and heightValid and toleranceValid
    
    def Reset(self):
        if(len(self.__path) >= 2):
            rospy.loginfo("CMissionLocal: reset to start")
            self.__curIdx = 0
        else:
            rospy.loginfo("CMissionLocal: failed to reset - invalid path")

    def Update(self):
        if(self.__curIdx >= len(self.__path)):
            # Last point is reached
            if(self.isBusy):
                self.isBusy = False
            return True
        
        if(not self.isBusy):
            self.isBusy = True

        curPos = [self.__movCtrl.pos.x, self.__movCtrl.pos.y]
        trgPt = self.__path[self.__curIdx]
        prevPt = self.__path[self.__curIdx - 1]

        # Set target position
        dx = trgPt[0] - curPos[0]
        dy = trgPt[1] - curPos[1]
        targetPosDelta = Vec3(dx, dy, 0.0)
        self.__movCtrl.SetPos(targetPosDelta.x + self.__movCtrl.pos.x, targetPosDelta.y + self.__movCtrl.pos.y, self.targetHeight)

        # Set target yaw
        dx = trgPt[0] - prevPt[0]
        dy = trgPt[1] - prevPt[1]
        targetYaw = np.arctan2(dy, dx)
        self.__movCtrl.SetYaw(targetYaw)
        
        xError = abs(curPos[0] - trgPt[0])
        yError = abs(curPos[1] - trgPt[1])
        
        if(xError <= self.__tolerance and yError <= self.__tolerance):
            rospy.loginfo("CMissionLocal::Update: passed waypoint #%d of #%d", self.__curIdx + 1, len(self.__path))
            self.__curIdx += 1

        return False

    def __onHeightChanged(self, height):
        self.targetHeight = height.value
        self.__movCtrl.SetParam("MIS_TAKEOFF_ALT", 0, self.targetHeight)
        rospy.loginfo("CMissionLocal: target height updated (%f)", self.targetHeight)

    def __onToleranceChanged(self, tolerance):
        self.__tolerance = tolerance.value
        rospy.loginfo("CMissionLocal: tolerance updated (%f)", self.__tolerance)