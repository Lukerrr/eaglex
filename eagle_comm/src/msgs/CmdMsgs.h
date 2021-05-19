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

DECLARE_MSG(CMD_HEIGHT, SCmdHeight,
    float height;
)

DECLARE_MSG(CMD_TOLERANCE, SCmdTolerance,
    float tolerance;
)