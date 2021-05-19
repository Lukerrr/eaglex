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
	
	CMD_MISSION,

	CMD_MAX,
};