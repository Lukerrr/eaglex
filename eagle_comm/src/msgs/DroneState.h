#pragma once

#include <stdint.h>

struct SDroneState
{
	enum ESystemState : uint8_t
	{
		ST_DISCONNECTED = 0,
		ST_IDLE,
		ST_TAKEOFF,
		ST_WORKING,
		ST_LANDING,
	};

	// Global position data
	double lat, lon, heading;
	bool bGlobalPos;

	// Position data
	float x, y, z;
	float roll, pitch, yaw;
	float groundDist;

	// Service data
	ESystemState systemState;
	uint32_t missionHash;
	bool bReady;
	bool bArmed;
	bool bOffboard;
	float charge;
};