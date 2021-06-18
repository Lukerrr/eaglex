#include "Communicator.h"
#include "CmdMsgs.h"
#include "Log.h"

#include "ros/ros.h"

#include <sys/ioctl.h>
#include <arpa/inet.h>

#define CONN_PORT_DEF 54000
#define CONN_MAX_CLIENTS 1

size_t GetCmdDataSize(ECmdType type)
{
    switch(type)
    {
    case CMD_ARM:               return sizeof(SCmdArm);
    case CMD_DISARM:            return sizeof(SCmdDisarm);
    case CMD_START:             return sizeof(SCmdStart);
    case CMD_STOP:              return sizeof(SCmdStop);
    case CMD_HEIGHT:            return sizeof(SCmdHeight);
    case CMD_TOLERANCE:         return sizeof(SCmdTolerance);
    case CMD_GET_CLOUD:         return sizeof(SCmdGetCloud);
    case CMD_MISSION:           return sizeof(SCmdMission);
    default:
        return 0;
    }
}

CCommunicator::CCommunicator()
{
    ros::NodeHandle nh("~");
    int port = nh.param("port", CONN_PORT_DEF);
    m_port = port;
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
            ECmdType cmdType;
            dataLen = RecvInternal(m_clSock, &cmdType, sizeof(cmdType));

            if (dataLen == 0)
            {
                ROS_INFO_NAMED(LOG_NAME, "Communicator: Ground station %d disconnected", m_clSock);

                // Client disconnected
                close(m_clSock);
                m_clSock = -1;
            }
            else if (dataLen > 0 && cmdType < CMD_MAX)
            {
                // Call command handler
                uint8_t *cmdData = NULL;
                size_t dataSize = GetCmdDataSize(cmdType);
                if(dataSize > 0)
                {
                    cmdData = new uint8_t[dataSize];
                    RecvInternal(m_clSock, cmdData, dataSize);
                }
                m_cmdHandler.Invoke(cmdType, cmdData);

                if(cmdData != NULL)
                {
                    delete[] cmdData;
                }

                ROS_INFO_NAMED(LOG_NAME, "Communicator: received command %d", cmdType);
            }
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
            ROS_INFO_NAMED(LOG_NAME, "Communicator: Connected to a ground station '%s:%d' (%d)",
                           inet_ntoa(clAddr.sin_addr), clAddr.sin_port, m_clSock);
        }
    }
}

int CCommunicator::RecvInternal(int socket, void* buf, size_t len)
{
    u_long availBytes = 0;
    int ioctlRes = ioctl(socket, FIONREAD, &availBytes);

    if(ioctlRes == -1 || availBytes == 0)
    {
        return -1;
    }

    if(len > availBytes)
    {
        ROS_WARN_NAMED(LOG_NAME, "Communicator: requested receive more bytes than available (%lu > %lu)", len, availBytes);
        len = availBytes;
    }
    
    return recv(socket, buf, len, 0);
}

void CCommunicator::SendInternal(char* pData, int len)
{
    if (!m_bIsValid || m_clSock == -1)
    {
        return;
    }

    if (send(m_clSock, pData, len, 0) == -1)
    {
        ROS_ERROR_NAMED(LOG_NAME, "Communicator: Cannot send message (%s).", strerror(errno));
        Initialize();
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

    sockaddr_in svSocketInfo;
    svSocketInfo.sin_family = AF_INET;
    svSocketInfo.sin_port = htons(m_port);
    svSocketInfo.sin_addr.s_addr = INADDR_ANY;

    int opt = 1;
    if (setsockopt(m_svSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof (opt)) == -1)
    {
        ROS_WARN_NAMED(LOG_NAME, "Communicator:%s: Set REUSEADDR error: %s", __func__, strerror(errno));
    }

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
        shutdown(m_svSock, SHUT_RDWR);
        close(m_svSock);
        m_svSock = -1;
    }

    if (m_clSock != -1)
    {
        shutdown(m_clSock, SHUT_RDWR);
        close(m_clSock);
        m_clSock = -1;
    }

    m_bIsValid = false;
}
