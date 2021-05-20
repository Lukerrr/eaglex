/*
* Response messages declaration
* Drone -> GroundStation
*/

#pragma once

#include "MsgDeclaration.h"

enum ESystemState : uint8_t
{
    ST_DISCONNECTED = 0,
    ST_IDLE,
    ST_TAKEOFF,
    ST_WORKING,
    ST_LANDING,
};

DECLARE_MSG(RSP_DRONE_STATE, SRspDroneState,

    // Global position data
    double lat;
    double lon;
    double heading;
    bool bGlobalPos;

    // Position data
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

    // Service data
    ESystemState systemState;
    uint32_t missionHash;
    uint32_t cloudSize;
    bool bReady;
    bool bArmed;
    bool bOffboard;
    float charge;
)

DECLARE_MSG(RSP_POINT_CLOUD, SRspPointCloud,
    struct CloudPt
    {
        double x;
        double y;
        double z;
    };

    uint32_t cloudSize;

    static const int cloudMaxSize = 64; 
    CloudPt cloud[cloudMaxSize];
)

typedef SRspDroneState SDroneState;
typedef SRspPointCloud SPointCloud;