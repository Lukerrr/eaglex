#include "UploadManager.h"
#include "Communicator.h"
#include "Log.h"

#include "RspMsgs.h"

void CUploadManager::BeginUpload(std::vector<geometry_msgs::Vector3> cloud)
{
    if(m_pointsNum > 0)
    {
        ROS_INFO_NAMED(LOG_NAME, "UploadManager: received a new point cloud while dispatching the old one. Ignoring...");
        return;
    }

    m_pointsNum = cloud.size();

    if(m_pointsNum == 0)
    {
        ROS_INFO_NAMED(LOG_NAME, "UploadManager: received an empty point cloud. Ignoring...");
        return;
    }

    ROS_INFO_NAMED(LOG_NAME, "UploadManager: received a cloud of %d points. Dispatching...", m_pointsNum);

    m_chunksNum = 0;
    m_cloud = cloud;

    if(m_bLocked)
    {
        m_bLocked = false;
    }
}

void CUploadManager::EndUpload()
{
    if(m_pointsNum <= 0)
    {
        return;
    }

    ROS_INFO_NAMED(LOG_NAME, "UploadManager: stopping upload...");

    m_pointsNum = 0;
    m_pointsNum = 0;
    m_cloud.clear();
}

void CUploadManager::Unlock()
{
    m_bLocked = false;
}

void CUploadManager::Update()
{
    if(m_pointsNum > 0 && !m_bLocked)
    {
        SPointCloud cloud;
        cloud.cloudSize = m_pointsNum;
        if(cloud.cloudSize > cloud.cloudMaxSize)
        {
            cloud.cloudSize = cloud.cloudMaxSize;
        }

        for(int i = 0; i < cloud.cloudSize; ++i)
        {
            geometry_msgs::Vector3 cloudPt = m_cloud[m_chunksNum * cloud.cloudMaxSize + i];
            cloud.cloud[i].x = cloudPt.x;
            cloud.cloud[i].y = cloudPt.y;
            cloud.cloud[i].z = cloudPt.z;
        }

        g_pComm->Send(cloud);

        ROS_INFO_NAMED(LOG_NAME, "CUploadManager: sent cloud chunk (%d points)", cloud.cloudSize);
        m_pointsNum -= cloud.cloudSize;
        ++m_chunksNum;

        if(m_pointsNum <= 0)
        {
            ROS_INFO_NAMED(LOG_NAME, "CUploadManager: point cloud was sent in %d chunks", m_chunksNum);
            m_pointsNum = 0;
            m_pointsNum = 0;
            m_cloud.clear();
        }
        else
        {
            m_bLocked = true;
        }
    }
}