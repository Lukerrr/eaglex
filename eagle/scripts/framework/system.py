#!/usr/bin/env python

import rospy

from helpers.ros_globals import *
from framework.gps import CGpsSystem
from framework.mission import CMission
from framework.mov_ctrl import CMovementController
from framework.point_cloud import CPointCloud
from framework.drone_listener import CDroneListener
from framework.system_state import ESystemState

from eagle_comm.msg import DroneState
from eagle_comm.msg import GsCmdSimple

class CSystem:
    def __init__(self):
        rospy.init_node("eagle_movement_system", anonymous = True)
        self.__systemState = ESystemState.DISCONNECTED
        self.__lastState = ESystemState.IDLE
        self.__rate = rospy.Rate(rospy.get_param("rate", default=20))
        self.__takeoffPos = None
        self.__landingPos = None
        self.__gps = CGpsSystem()
        self.__movCtrl = CMovementController(self.__gps)
        self.__pointCloud = CPointCloud(self.__movCtrl)
        self.__mission = CMission(self.__movCtrl, self.__gps)
        self.__droneLst = CDroneListener(self.__movCtrl, self.__gps, self.__mission)
        self.__armSub = rospy.Subscriber("eagle_comm/in/cmd_arm", GsCmdSimple, self.__onCmdArm)
        self.__disarmSub = rospy.Subscriber("eagle_comm/in/cmd_disarm", GsCmdSimple, self.__onCmdDisarm)
        self.__armSub = rospy.Subscriber("eagle_comm/in/cmd_start", GsCmdSimple, self.__onCmdStart)
        self.__disarmSub = rospy.Subscriber("eagle_comm/in/cmd_stop", GsCmdSimple, self.__onCmdStop)
        self.__getCloudSub = rospy.Subscriber("eagle_comm/in/cmd_get_cloud", GsCmdSimple, self.__onCmdGetCloud)
        self.__stateModes = \
        {\
            ESystemState.TAKEOFF: "OFFBOARD",\
            ESystemState.WORKING: "OFFBOARD",\
            ESystemState.LANDING: "OFFBOARD"\
        }

    def Run(self):
        rospy.loginfo("CSystem: waiting for a timer...")
        # Wait for a timer valid response
        while(True):
            self.__time = now().to_sec()
            self.__rate.sleep()
            if(self.__time > 0.0):
                break

        rospy.loginfo("CSystem: begin working loop")
        # Working loop
        while(ok()):
            # Get delta time
            curTime = now().to_sec()
            dt = curTime - self.__time
            self.__time = curTime

            # Detect auto-disarm
            if(not self.__movCtrl.simState.armed and self.__systemState.value > ESystemState.IDLE.value):
                rospy.logwarn("CSystem: disarmed by autopilot!")
                self.__setState(ESystemState.IDLE)

            isWorking = self.__systemState.value > ESystemState.IDLE.value
                
            # Prevent mission changes if working
            self.__mission.isLocked = isWorking

            # Validate mission if working
            if(isWorking and not self.__mission.IsValid()):
                rospy.logwarn("CSystem: mission is invalid, landing...")
                self.__setState(ESystemState.LANDING)

            # Check is connected
            if(self.__systemState != ESystemState.DISCONNECTED):
                if(self.__movCtrl.simState.connected == False):
                    rospy.logwarn("CSystem: disconnected from FMU!")
                    self.__lastState = self.__systemState
                    if(self.__lastState != ESystemState.IDLE):
                        rospy.loginfo("CSystem: reserved state '%s'", self.__lastState.name)
                    self.__setState(ESystemState.DISCONNECTED)

            # Check is flight mode valid
            requiredMode = self.__stateModes.get(self.__systemState)
            curMode = self.__movCtrl.simState.mode
            if(requiredMode != None and curMode != requiredMode):
                self.__movCtrl.SetMode(requiredMode)

            if(dt > 0.0):
                # State DISCONNECTED
                if(self.__systemState == ESystemState.DISCONNECTED):
                    if(self.__movCtrl.simState.connected == True):
                        self.__setState(self.__lastState)
                        if(self.__systemState != ESystemState.IDLE):
                            rospy.loginfo("CSystem: restored state '%s'", self.__systemState.name)

                # State IDLE
                elif(self.__systemState == ESystemState.IDLE):
                    pass

                # State TAKEOFF
                elif(self.__systemState == ESystemState.TAKEOFF):
                    if(self.__takeoffPos == None):
                        self.__takeoffPos = [self.__movCtrl.pos.x, self.__movCtrl.pos.y]
                    self.__movCtrl.SetPos(self.__takeoffPos[0], self.__takeoffPos[1], self.__mission.targetHeight)
                    if(self.__movCtrl.pos.z >= self.__mission.targetHeight * 0.8):
                        rospy.loginfo("CSystem: takeoff was done at height %f", self.__movCtrl.pos.z)
                        self.__setState(ESystemState.WORKING)
                        self.__mission.Reset()
                        self.__takeoffPos = None

                # State WORKING
                elif(self.__systemState == ESystemState.WORKING):
                    if(self.__mission.Update()):
                        rospy.loginfo("CSystem: mission is done, landing...")
                        self.__setState(ESystemState.LANDING)

                # State LANDING
                elif(self.__systemState == ESystemState.LANDING):
                    if(self.__landingPos == None):
                        self.__landingPos = [self.__movCtrl.pos.x, self.__movCtrl.pos.y]
                    self.__movCtrl.SetPos(self.__landingPos[0], self.__landingPos[1], 0.0)
                    if(self.__movCtrl.pos.z <= 0.15):
                        rospy.loginfo("CSystem: landing was done! Disarming...")
                        self.__movCtrl.SetIsArmed(False)
                        self.__setState(ESystemState.IDLE)
                        self.__landingPos = None
            else:
                rospy.logwarn("CSystem: delta time was invalid (%f) at %f sec.", dt, curTime)

            self.__movCtrl.DispatchPosition()
            self.__droneLst.SendData(self.__systemState.value, self.__pointCloud.GetSize())
            self.__rate.sleep()


    def Exit(self):
        self.__movCtrl.SetMode("AUTO.LAND")

    def __setState(self, st):
        if(self.__systemState != st):
            rospy.loginfo("CSystem::setState: %s -> %s", self.__systemState.name, st.name)
            self.__systemState = st
        else:
            rospy.loginfo("CSystem::setState: '%s' state loop ignored", st.name)
    
    def __onCmdArm(self, cmd):
        rospy.loginfo("CSystem: command 'ARM' received")
        if(not self.__movCtrl.simState.armed):
            self.__movCtrl.SetIsArmed(True)
        else:
            rospy.loginfo("CSystem: already armed")

    def __onCmdDisarm(self, cmd):
        rospy.loginfo("CSystem: command 'DISARM' received")
        if(self.__systemState.value > ESystemState.IDLE.value):
            rospy.loginfo("CSystem: cannot disarm if working. Starting land...")
            self.__setState(ESystemState.LANDING)
        else:
            if(self.__movCtrl.simState.armed):
                self.__movCtrl.SetIsArmed(False)
            else:
                rospy.loginfo("CSystem: already disarmed")
    
    def __onCmdStart(self, cmd):
        rospy.loginfo("CSystem: command 'START' received")
        if(self.__systemState == ESystemState.IDLE):
            if(self.__movCtrl.simState.armed):
                self.__setState(ESystemState.TAKEOFF)
            else:
                rospy.loginfo("CSystem: cannot work if disarmed")
        else:
            rospy.loginfo("CSystem: cannot work from state '%s'", self.__systemState.name)

    def __onCmdStop(self, cmd):
        rospy.loginfo("CSystem: command 'STOP' received")
        if(self.__systemState.value > ESystemState.IDLE.value):
            if(self.__movCtrl.simState.armed):
                self.__setState(ESystemState.LANDING)
            else:
                rospy.loginfo("CSystem: cannot stop if disarmed")
        else:
            rospy.loginfo("CSystem: invalid working state '%s'", self.__systemState.name)
    
    def __onCmdGetCloud(self, cmd):
        rospy.loginfo("CSystem: command 'GET_CLOUD' received")
        self.__pointCloud.PublishCloud()