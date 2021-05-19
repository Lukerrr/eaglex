#pragma once

#include "CommandHandler.h"
#include <MsgTypes.h>
#include <DroneState.h>

class CCommunicator
{
public:
	CCommunicator(uint16_t port, uint32_t maxDataLen);
	~CCommunicator();

	void Update();
	void SendDroneState(SDroneState state);

private:
	bool Initialize();
	void Invalidate();

	uint16_t m_port;
	uint32_t m_maxDataLen;

	CCommandHandler m_cmdHandler;

	int m_svSock = -1;
	int m_clSock = -1;

	bool m_bIsValid = false;
};