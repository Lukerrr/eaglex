#include "Communicator.h"
#include "Log.h"

#include <sys/fcntl.h>
#include <arpa/inet.h>

#define CONN_MAX_CLIENTS 1

CCommunicator::CCommunicator(uint16_t port, uint32_t maxDataLen)
{
    m_port = port;
    m_maxDataLen = maxDataLen;
}

CCommunicator::~CCommunicator()
{
    Invalidate();
}

void CCommunicator::Update()
{
    if (!m_bIsValid)
    {
        if (!Initialize())
        {
            return;
        }
    }

    if (m_clSock != -1)
    {
        // Read all received packets
        int dataLen = 0;
        do
        {
            char *data = new char[m_maxDataLen];
            memset(data, 0, m_maxDataLen);
            dataLen = recv(m_clSock, data, m_maxDataLen, 0);

            if (dataLen == 0)
            {
                ROS_INFO_NAMED(LOG_NAME, "Communicator: Ground station %d disconnected", m_clSock);

                // Client disconnected
                close(m_clSock);
                m_clSock = -1;
            }
            else if (dataLen > 0)
            {
                // Call command handler
                ECmdType cmdType = *(ECmdType *)data;
                void *cmdData = dataLen > 1 ? data + sizeof(ECmdType) : NULL;
                m_cmdHandler.Invoke(cmdType, cmdData);

                ROS_INFO_NAMED(LOG_NAME, "Communicator: received command %d", cmdType);
            }

            delete[] data;

        } while (dataLen > 0);
    }
    else
    {
        // Waiting for a new client

        sockaddr_in clAddr;
        socklen_t clAddrLen = sizeof(clAddr);

        m_clSock = accept(m_svSock, (sockaddr *)&clAddr, &clAddrLen);

        if (m_clSock != -1)
        {
            fcntl(m_clSock, F_SETFL, fcntl(m_clSock, F_GETFL, 0) | O_NONBLOCK);
            ROS_INFO_NAMED(LOG_NAME, "Communicator: Connected to a ground station '%s:%d' (%d)",
                           inet_ntoa(clAddr.sin_addr), clAddr.sin_port, m_clSock);
        }
    }
}

void CCommunicator::SendInternal(char* pData, int len)
{
    if (!m_bIsValid || m_clSock == -1)
    {
        return;
    }
    
    if (send(m_clSock, pData, len, 0) == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator: Cannot send a state (%s). Closing connection...", strerror(errno));
        Invalidate();
    }
}

bool CCommunicator::Initialize()
{
    Invalidate();

    m_svSock = socket(AF_INET, SOCK_STREAM, 0);
    if (m_svSock == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator:%s: Socket error: %s", __func__, strerror(errno));
        return false;
    }

    fcntl(m_svSock, F_SETFL, fcntl(m_svSock, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in svSocketInfo;
    svSocketInfo.sin_family = AF_INET;
    svSocketInfo.sin_port = htons(m_port);
    svSocketInfo.sin_addr.s_addr = INADDR_ANY;
    if (bind(m_svSock, (sockaddr *)&svSocketInfo, sizeof(svSocketInfo)) == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator:%s: Bind error: %s", __func__, strerror(errno));
        return false;
    }

    if (listen(m_svSock, CONN_MAX_CLIENTS) == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator:%s: Listen error: %s", __func__, strerror(errno));
        return false;
    }

    ROS_INFO_NAMED(LOG_NAME, "Communicator: successfully intialized! Socket = %d", m_svSock);

    m_bIsValid = true;

    return true;
}

void CCommunicator::Invalidate()
{
    ROS_INFO_NAMED(LOG_NAME, "Communicator: invalidating...");

    if (m_svSock != -1)
    {
        close(m_svSock);
        m_svSock = -1;
    }

    if (m_clSock != -1)
    {
        close(m_clSock);
        m_clSock = -1;
    }

    m_bIsValid = false;
}
