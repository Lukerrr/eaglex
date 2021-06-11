#include "CommandHandler.h"
#include "CmdMsgs.h"
#include "Log.h"

#include "ros/ros.h"
#include "eagle_comm/GsCmdSimple.h"
#include "eagle_comm/GsCmdMission.h"
#include "eagle_comm/GsCmdFloat.h"

#include <string.h>

static ros::Publisher s_armPub;
static ros::Publisher s_disarmPub;
static ros::Publisher s_startPub;
static ros::Publisher s_stopPub;
static ros::Publisher s_heightPub;
static ros::Publisher s_tolerancePub;
static ros::Publisher s_getCloudBeginPub;
static ros::Publisher s_getCloudNextPub;
static ros::Publisher s_getCloudEndPub;
static ros::Publisher s_missionPub;

void SendSimpleCmd(ros::Publisher& pub)
{
    eagle_comm::GsCmdSimple msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
}

void SendFloatCmd(ros::Publisher& pub, float val)
{
    eagle_comm::GsCmdFloat msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();
    msg.value = val;
    pub.publish(msg); 
}

void OnCmdArm(void* data)
{
    SendSimpleCmd(s_armPub);
}

void OnCmdDisarm(void* data)
{
    SendSimpleCmd(s_disarmPub);
}

void OnCmdStart(void* data)
{
    SendSimpleCmd(s_startPub);
}

void OnCmdStop(void* data)
{
    SendSimpleCmd(s_stopPub);
}

void OnCmdGetCloudBegin(void* data)
{
    SendSimpleCmd(s_getCloudBeginPub);
}

void OnCmdGetCloudNext(void* data)
{
    SendSimpleCmd(s_getCloudNextPub);
}

void OnCmdGetCloudEnd(void* data)
{
    SendSimpleCmd(s_getCloudEndPub);
}

void OnCmdHeight(void* data)
{
    SendFloatCmd(s_heightPub, *((float*)data));
}

void OnCmdTolerance(void* data)
{
    SendFloatCmd(s_tolerancePub, *((float*)data));
}

void OnCmdMission(void* data)
{
    eagle_comm::GsCmdMission msg;
    msg.header = std_msgs::Header();
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();

    SMissionData missionData = *((SMissionData*)data);

    for(int i = 0; i < missionData.pathSize; ++i)
    {
        geometry_msgs::Pose2D pose;
        pose.x = missionData.path[i].x;
        pose.y = missionData.path[i].y;
        pose.theta = 0.0;
        msg.path.push_back(pose);
    }

    msg.hash = missionData.hash;
    s_missionPub.publish(msg);
}

CCommandHandler::CCommandHandler()
{
    m_handlers[CMD_ARM] = OnCmdArm;
    m_handlers[CMD_DISARM] = OnCmdDisarm;
    m_handlers[CMD_START] = OnCmdStart;
    m_handlers[CMD_STOP] = OnCmdStop;
    m_handlers[CMD_HEIGHT] = OnCmdHeight;
    m_handlers[CMD_TOLERANCE] = OnCmdTolerance;
    m_handlers[CMD_GET_CLOUD_BEGIN] = OnCmdGetCloudBegin;
    m_handlers[CMD_GET_CLOUD_NEXT] = OnCmdGetCloudNext;
    m_handlers[CMD_GET_CLOUD_END] = OnCmdGetCloudEnd;
    m_handlers[CMD_MISSION] = OnCmdMission;
    
    ros::NodeHandle nh("~");
    s_armPub = nh.advertise<eagle_comm::GsCmdSimple>("in/cmd_arm", 1);
    s_disarmPub = nh.advertise<eagle_comm::GsCmdSimple>("in/cmd_disarm", 1);
    s_startPub = nh.advertise<eagle_comm::GsCmdSimple>("in/cmd_start", 1);
    s_stopPub = nh.advertise<eagle_comm::GsCmdSimple>("in/cmd_stop", 1);
    s_heightPub = nh.advertise<eagle_comm::GsCmdFloat>("in/cmd_height", 1);
    s_tolerancePub = nh.advertise<eagle_comm::GsCmdFloat>("in/cmd_tolerance", 1);
    s_getCloudBeginPub = nh.advertise<eagle_comm::GsCmdSimple>("in/cmd_get_cloud", 1);
    s_getCloudNextPub = nh.advertise<eagle_comm::GsCmdSimple>("in/cmd_get_cloud_next", 1);
    s_getCloudEndPub = nh.advertise<eagle_comm::GsCmdSimple>("in/cmd_get_cloud_end", 1);
    s_missionPub = nh.advertise<eagle_comm::GsCmdMission>("in/cmd_mission", 1);
}

CCommandHandler::~CCommandHandler()
{
    memset(m_handlers, 0, sizeof(m_handlers));
}

void CCommandHandler::Invoke(ECmdType type, void* data)
{
    if(type < CMD_MAX)
    {
        m_handlers[type](data);
    }
}