#!/usr/bin/env python

import rospy

from eagle_comm.msg import GsCmdMission
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

        self.__path = []
        self.__curIdx = 0

        self.targetHeight = 0.0
        self.__tolerance = 0.0

        self.__missionSub = rospy.Subscriber("eagle_comm/in/cmd_mission", GsCmdMission, self.__onMissionChanged)
        self.__heightSub = rospy.Subscriber("eagle_comm/in/cmd_height", GsCmdFloat, self.__onHeightChanged)
        self.__toleranceSub = rospy.Subscriber("eagle_comm/in/cmd_tolerance", GsCmdFloat, self.__onToleranceChanged)

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
    
    def Reset(self):
        rospy.loginfo("CMission: reset to start")
        # Set current position as the return point
        self.__path[-1] = [self.__gps.lat, self.__gps.lon]
        self.__curIdx = 0

    def Update(self):
        if(self.__curIdx >= len(self.__path)):
            # Last point is reached
            if(self.isBusy):
                self.isBusy = False
            return True
        
        if(not self.isBusy):
            self.isBusy = True

        trgPos = self.__path[self.__curIdx]
        lat1 = self.__gps.lat
        lon1 = self.__gps.lon
        lat2 = trgPos[0]
        lon2 = trgPos[1]

        dx = 1000.0*(lon2-lon1)*40000*np.cos((lat1+lat2)*np.pi/360)/360
        dy = 1000.0*(lat2-lat1)*40000/360

        targetPosDelta = Vec3(dx, dy, 0.0)
        self.__movCtrl.SetPos(targetPosDelta.x + self.__movCtrl.pos.x, targetPosDelta.y + self.__movCtrl.pos.y, self.targetHeight)

        targetYaw = np.arctan2(targetPosDelta.y, targetPosDelta.x)
        self.__movCtrl.SetYaw(targetYaw)
        
        latError = abs(self.__gps.lat - trgPos[0])
        lonError = abs(self.__gps.lon - trgPos[1])

        # Convert tolerance from meters to degrees
        tol = np.degrees(self.__tolerance / (abs(np.cos(np.radians(trgPos[0]))) * 6371032.0))
        
        if(latError <= tol and lonError <= tol):
            rospy.loginfo("CMission::Update: passed waypoint #%d of #%d", self.__curIdx + 1, len(self.__path))
            self.__curIdx += 1

        return False
    
    def __onMissionChanged(self, mission):
        if(self.isLocked):
            rospy.logwarn("CMission: data received, but mission is locked")
        else:
            self.hash = mission.hash
            self.__path = []
            for ptGeo in mission.path:
                self.__path.append([ptGeo.x, ptGeo.y])
            self.__path.append([self.__gps.lat, self.__gps.lon])
            rospy.loginfo("CMission: mission updated (hash = %#08x)", self.hash)

    def __onHeightChanged(self, height):
        self.targetHeight = height.value
        self.__movCtrl.SetParam("MIS_TAKEOFF_ALT", 0, self.targetHeight)
        rospy.loginfo("CMission: target height updated (%f)", self.targetHeight)

    def __onToleranceChanged(self, tolerance):
        self.__tolerance = tolerance.value
        rospy.loginfo("CMission: tolerance updated (%f)", self.__tolerance)