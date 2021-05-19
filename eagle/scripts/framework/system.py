#!/usr/bin/env python

import rospy

from helpers.ros_globals import *
from framework.config import g_config
from framework.gps import CGpsSystem
from framework.mission import CMission
from framework.mov_ctrl import CMovementController
from framework.drone_listener import CDroneListener
from framework.system_state import ESystemState

from eagle_comm.msg import DroneState
from eagle_comm.msg import GsCmdSimple

class CSystem:
    def __init__(self):
        rospy.init_node("eagle_movement_system", anonymous = True)
        self.__systemState = ESystemState.DISCONNECTED
        self.__lastState = ESystemState.IDLE
        self.__rate = rospy.Rate(1.0 / g_config.deltaSeconds)
        self.__gps = CGpsSystem()
        self.__movCtrl = CMovementController(self.__gps)
        self.__mission = CMission(self.__movCtrl, self.__gps)
        self.__droneLst = CDroneListener(self.__movCtrl, self.__gps, self.__mission)
        self.__armSub = rospy.Subscriber("eagle_comm/in/cmd_arm", GsCmdSimple, self.__onCmdArm)
        self.__disarmSub = rospy.Subscriber("eagle_comm/in/cmd_disarm", GsCmdSimple, self.__onCmdDisarm)
        self.__armSub = rospy.Subscriber("eagle_comm/in/cmd_start", GsCmdSimple, self.__onCmdStart)
        self.__disarmSub = rospy.Subscriber("eagle_comm/in/cmd_stop", GsCmdSimple, self.__onCmdStop)

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

            if(not self.__movCtrl.simState.armed and self.__systemState.value > ESystemState.IDLE.value):
                rospy.logwarn("CSystem: disarmed by autopilot!")
                self.__setState(ESystemState.IDLE)

            self.__mission.isLocked = self.__systemState.value > ESystemState.IDLE.value

            if(self.__systemState != ESystemState.DISCONNECTED):
                if(self.__movCtrl.simState.connected == False):
                    rospy.logwarn("CSystem: disconnected from FMU!")
                    self.__setState(ESystemState.DISCONNECTED)
                elif(not self.__isOffboard()):
                    if(self.__systemState != ESystemState.SETUP_MODE):
                        rospy.logwarn("CSystem: OFFBOARD was disabled, resetting...")
                        self.__lastState = self.__systemState
                        self.__setState(ESystemState.SETUP_MODE)

            if(dt > 0.0):
                # State DISCONNECTED
                if(self.__systemState == ESystemState.DISCONNECTED):
                    if(self.__movCtrl.simState.connected == True):
                        if(not self.__isOffboard()):
                            self.__setState(ESystemState.SETUP_MODE)
                        else:
                            self.__setState(self.__lastState)

                # State SETUP_MODE
                elif(self.__systemState == ESystemState.SETUP_MODE):
                    if(not self.__isOffboard()):
                        self.__movCtrl.SetMode("OFFBOARD")
                    else:
                        self.__setState(ESystemState.IDLE)

                # State IDLE
                elif(self.__systemState == ESystemState.IDLE):
                    pass

                # State TAKEOFF
                elif(self.__systemState == ESystemState.TAKEOFF):
                    self.__movCtrl.SetVel(0, 0, g_config.tkoffVelocity)
                    if(self.__movCtrl.height >= g_config.tkoffHeight):
                        rospy.loginfo("CSystem: takeoff was done at height %f", self.__movCtrl.height)
                        self.__setState(ESystemState.WORKING)
                        self.__mission.Reset()

                # State WORKING
                elif(self.__systemState == ESystemState.WORKING):
                    if(self.__mission.Update()):
                        rospy.loginfo("CSystem: mission is done, landing...")
                        self.__setState(ESystemState.LANDING)

                # State LANDING
                elif(self.__systemState == ESystemState.LANDING):
                    self.__movCtrl.SetVel(0, 0, -g_config.landVelocity)
                    if(self.__movCtrl.height <= g_config.landVelocity):
                        rospy.loginfo("CSystem: landing was done! Disarming...")
                        self.__movCtrl.SetIsArmed(False)
                        self.__setState(ESystemState.IDLE)
            else:
                rospy.logwarn("CSystem: delta time was invalid (%f) at %f sec.", dt, curTime)

            self.__movCtrl.DispatchPosition()
            self.__droneLst.SendData(self.__systemState.value)
            self.__rate.sleep()


    def Exit(self):
        self.__movCtrl.SetMode("AUTO.LAND")
        rospy.sleep(1)

    def __isOffboard(self):
        return self.__movCtrl.simState.mode == "OFFBOARD"

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