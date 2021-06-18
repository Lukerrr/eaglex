#pragma once

#include "Singleton.h"
#include "CommandHandler.h"
#include "RspMsgs.h"

#define g_pComm ((&CCommunicator::Instance()))

class CCommunicator : public TSingleton<CCommunicator>
{
public:
    CCommunicator();
    ~CCommunicator();

    void Update();

    template<typename T>
    void Send(T msg);

private:
    bool Initialize();
    void Invalidate();

    int RecvInternal(int socket, void* buf, size_t len);
    void SendInternal(char* pData, int len);

    uint16_t m_port;

    CCommandHandler m_cmdHandler;

    int m_svSock = -1;
    int m_clSock = -1;

    bool m_bIsValid = false;
};

template<typename T>
void CCommunicator::Send(T msg)
{
    int len = sizeof(T) + sizeof(ERspType);
    char* buf = new char[len];
    memcpy(buf, &T::s_type, sizeof(ERspType));
    memcpy(buf + sizeof(ERspType), &msg, sizeof(T));
    SendInternal(buf, len);
    delete[] buf;
}