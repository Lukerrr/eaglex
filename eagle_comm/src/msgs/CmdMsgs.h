/*
* Command messages declaration
* GroundStation -> Drone
*/

#pragma once

#include "MsgDeclaration.h"

DECLARE_MSG_NO_PARAMS(CMD_ARM, SCmdArm)
DECLARE_MSG_NO_PARAMS(CMD_DISARM, SCmdDisarm)
DECLARE_MSG_NO_PARAMS(CMD_START, SCmdStart)
DECLARE_MSG_NO_PARAMS(CMD_STOP, SCmdStop)
DECLARE_MSG_NO_PARAMS(CMD_GET_CLOUD, SCmdGetCloud)

DECLARE_MSG(CMD_HEIGHT, SCmdHeight,
    float height;
)

DECLARE_MSG(CMD_TOLERANCE, SCmdTolerance,
    float tolerance;
)

DECLARE_MSG(CMD_MISSION, SCmdMission,
    struct Point
    {
        double x;
        double y;
    };

    uint32_t hash;
    uint32_t pathSize;
    
    static const int pathMaxSize = 128; 
    Point path[pathMaxSize];
)

typedef SCmdMission SMissionData;