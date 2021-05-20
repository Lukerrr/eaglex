#!/usr/bin/env python

import rospy

from framework.mov_ctrl import CMovementController

from sensor_msgs.msg import BatteryState
from eagle_comm.msg import DroneState

class CDroneListener:
    def __init__(self, movCtrl, gps, mission):
        self.__gps = gps
        self.__mission = mission
        self.__movCtrl = movCtrl
        self.__batCharge = 0.0
        self.__chargeSub = rospy.Subscriber("mavros/battery", BatteryState, self.__onBatteryStateChanged)
        self.__droneStatePub = rospy.Publisher("eagle_comm/out/drone_state", DroneState, queue_size = 1)
    
    def SendData(self, systemState, cloudSize):
        state = DroneState()
        state.lat = self.__gps.lat
        state.lon = self.__gps.lon
        state.heading = self.__gps.heading
        state.globalPos = 1
        state.pos.x = self.__movCtrl.pos.x
        state.pos.y = self.__movCtrl.pos.y
        state.pos.z = self.__movCtrl.pos.z
        state.angles.x = self.__movCtrl.rot.roll
        state.angles.y = self.__movCtrl.rot.pitch
        state.angles.z = self.__movCtrl.rot.yaw
        state.missionHash = self.__mission.hash
        state.charge = self.__batCharge
        state.systemState = systemState
        state.cloudSize = cloudSize
        state.armed = self.__movCtrl.simState.armed
        state.offboard = (self.__movCtrl.simState.mode == "OFFBOARD")
        self.__droneStatePub.publish(state)
    
    def __onBatteryStateChanged(self, bat):
        if(bat.present):
            self.__batCharge = bat.percentage
        else:
            self.__batCharge = 0.0