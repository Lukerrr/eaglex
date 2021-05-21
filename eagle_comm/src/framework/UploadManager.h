#pragma once

#include "geometry_msgs/Vector3.h"

class CUploadManager
{
public:
    void BeginUpload(std::vector<geometry_msgs::Vector3> cloud);
    void Update();

private:
    int m_pointsNum = 0;
    int m_chunksNum = 0;
    std::vector<geometry_msgs::Vector3> m_cloud;
};