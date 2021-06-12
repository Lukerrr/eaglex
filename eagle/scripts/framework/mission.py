#!/usr/bin/env python3

import rospy

from eagle_comm.msg import GsCmdMission
from eagle_comm.msg import GsCmdFloat

from helpers.math_utils import *

import numpy as np

################################################################################
## A class to store the mission and execute quadrotor's movement
## along the given path in geographical coordinates.
##
## Fields list:
###
##  isLocked - whether or not the path changes are locked (e.g. during flight)
##  isBusy - whether or not the mission is currently executed
##  hash - current mission crc32 hash number for validation
##  targetHeight - flight height in meters
##  __tolerance - distance to reach path points in meters
##  __path - mission geographical points array
##  __curIdx - target point index
##  __gps - CGpsSystem instance ref to receive a global position
##  __movCtrl - CMovementController instance ref to drive the quadrotor
###
##  __missionSub - mission path topic subscriber
##  __heightSub - target height topic subscriber
##  __toleranceSub - tolerance topic subscriber
##
##
## Methods list:
##  IsValid - returns true if the path and the mission parameters are valid
##  Reset - resets mission to start
##  Update - updates the mission execution
##  __onMissionChanged - mission path topic callback
##  __onHeightChanged - target height topic callback
##  __onToleranceChanged - tolerance topic callback
################################################################################
class CMission:
    def __init__(self, movCtrl, gps):
        self.isLocked = True
        self.isBusy = False
        self.hash = 0xFFFFFFFF

        self.__gps = gps
        self.__movCtrl = movCtrl

        self.__path = []
        self.__curIdx = 0

        self.targetHeight = 0.0
        self.__tolerance = 0.0

        self.__missionSub = rospy.Subscriber("eagle_comm/in/cmd_mission", GsCmdMission, self.__onMissionChanged)
        self.__heightSub = rospy.Subscriber("eagle_comm/in/cmd_height", GsCmdFloat, self.__onHeightChanged)
        self.__toleranceSub = rospy.Subscriber("eagle_comm/in/cmd_tolerance", GsCmdFloat, self.__onToleranceChanged)

    # Returns true if the path and the mission parameters are valid
    def IsValid(self):
        hashValid = self.hash != 0xFFFFFFFF
        pathValid = len(self.__path) >= 2
        heightValid = self.targetHeight >= 1.5
        toleranceValid = self.__tolerance > 0.0
        
        if(not hashValid):
            rospy.logwarn("CMission::IsValid: invalid mission hash")

        if(not pathValid):
            rospy.logwarn("CMission::IsValid: invalid path ('%s')", str(self.__path))
        
        if(not heightValid):
            rospy.logwarn("CMission::IsValid: invalid target height (%f)", self.targetHeight)
        
        if(not toleranceValid):
            rospy.logwarn("CMission::IsValid: invalid tolerance (%f)", self.__tolerance)

        return hashValid and pathValid and heightValid and toleranceValid

    # Reset mission to start
    def Reset(self):
        if(len(self.__path) >= 2):
            rospy.loginfo("CMission: reset to start")
            # Set current position as the return point
            self.__path[-1] = [self.__gps.lat, self.__gps.lon]
            self.__curIdx = 0
        else:
            rospy.loginfo("CMission: failed to reset - invalid path")

    ## Update the mission execution
    def Update(self):
        if(self.__curIdx >= len(self.__path)):
            # Last point is reached
            if(self.isBusy):
                self.isBusy = False
            return True
        
        if(not self.isBusy):
            self.isBusy = True

        curPos = [self.__gps.lat, self.__gps.lon]
        trgPt = self.__path[self.__curIdx]
        prevPt = self.__path[self.__curIdx - 1]

        # Set target position
        dx, dy = GetCartesianOffset(curPos, trgPt)
        targetPosDelta = Vec3(dx, dy, 0.0)
        self.__movCtrl.SetPos(targetPosDelta.x + self.__movCtrl.pos.x, targetPosDelta.y + self.__movCtrl.pos.y, self.targetHeight)

        # Set target yaw
        dx, dy = GetCartesianOffset(prevPt, trgPt)
        targetYaw = np.arctan2(dy, dx)
        self.__movCtrl.SetYaw(targetYaw)
        
        latError = abs(curPos[0] - trgPt[0])
        lonError = abs(curPos[1] - trgPt[1])

        # Convert tolerance from meters to degrees
        tol = MetersToGeoDegrees(self.__tolerance, trgPt[0])
        
        if(latError <= tol and lonError <= tol):
            rospy.loginfo("CMission::Update: passed waypoint #%d of #%d", self.__curIdx + 1, len(self.__path))
            self.__curIdx += 1

        return False

    ## Mission path topic callback
    def __onMissionChanged(self, mission):
        if(self.isLocked):
            rospy.logwarn("CMission: data received, but mission is locked")
        else:
            self.hash = mission.hash
            self.__path = []
            for ptGeo in mission.path:
                self.__path.append([ptGeo.x, ptGeo.y])
            # An extra point for return to home position
            self.__path.append([self.__gps.lat, self.__gps.lon])
            rospy.loginfo("CMission: mission updated (hash = %#08x)", self.hash)

    ## Target height topic callback
    def __onHeightChanged(self, height):
        self.targetHeight = height.value
        self.__movCtrl.SetParam("MIS_TAKEOFF_ALT", 0, self.targetHeight)
        rospy.loginfo("CMission: target height updated (%f)", self.targetHeight)

    ## Tolerance topic callback
    def __onToleranceChanged(self, tolerance):
        self.__tolerance = tolerance.value
        rospy.loginfo("CMission: tolerance updated (%f)", self.__tolerance)