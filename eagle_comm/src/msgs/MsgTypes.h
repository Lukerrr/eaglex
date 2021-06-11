#pragma once

#include <stdint.h>

typedef uint8_t EMsgType;

enum ECmdType : EMsgType
{
    CMD_ARM = 0,
    CMD_DISARM,
    CMD_START,
    CMD_STOP,
    CMD_HEIGHT,
    CMD_TOLERANCE,
    CMD_GET_CLOUD_BEGIN,
    CMD_GET_CLOUD_NEXT,
    CMD_GET_CLOUD_END,

    CMD_MISSION,

    CMD_MAX,
};

enum ERspType : EMsgType
{
    RSP_DRONE_STATE = 0,
    RSP_POINT_CLOUD,

    RSP_MAX,
};