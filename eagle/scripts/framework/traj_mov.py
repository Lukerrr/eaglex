
import rospy

from framework.mov_ctrl import CMovementController
from helpers.math_utils import *

class CTrajectoryMovement():
    def __init__(self, movCtrl, gps):
        self.__movCtrl = movCtrl
        self.__gps = gps
        self.__path = []
        self.__height = 0.0
        self.__tolerance = 0.0
        self.__curIdx = 0
        

    def Setup(self, path, height, tolerance):
        self.__path = path
        self.__height = height
        self.__tolerance = tolerance
        self.__curIdx = 0
        self.isBusy = True


    ## Returns true if last point is reached
    def Update(self):
        if(self.__curIdx >= len(self.__path)):
            # Last point is reached
            if(self.isBusy):
                self.isBusy = False
            return True

        trgPos = self.__path[self.__curIdx]
        
        self.__movCtrl.SetGeoPos(trgPos[0], trgPos[1], __height)

        ##
        # TODO: heading
        ##
        
        latError = abs(self.__gps.lat - self.__path[self.__curIdx][0])
        lonError = abs(self.__gps.lon - self.__path[self.__curIdx][1])

        if(latError <= self.__tolerance and lonError <= self.__tolerance):
            self.__curIdx += 1

        return False
