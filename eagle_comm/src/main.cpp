#include "framework/Communicator.h"
#include "framework/Log.h"

#include "ros/ros.h"
#include "eagle_comm/DroneState.h"
#include "eagle_comm/PointCloud.h"
#include "eagle_comm/GsCmdSimple.h"

#include <sstream>

#define COMM_RATE_DEF 10

void OnDroneStateUpdated(const eagle_comm::DroneState& msg)
{
    SDroneState state;

    state.lat = msg.lat;
    state.lon = msg.lon;
    state.heading = msg.heading;
    state.bGlobalPos = msg.globalPos != 0;

    state.x = msg.pos.x;
    state.y = msg.pos.y;
    state.z = msg.pos.z;

    state.roll = msg.angles.x;
    state.pitch = msg.angles.y;
    state.yaw = msg.angles.z;

    state.systemState = (ESystemState)msg.systemState;
    state.missionHash = msg.missionHash;
    state.cloudSize = msg.cloudSize;

    state.bReady = msg.ready != 0;
    state.bArmed = msg.armed != 0;
    state.bOffboard = msg.offboard != 0;
    state.charge = msg.charge;

    g_pComm->Send(state);
}

void OnPointCloudReceived(const eagle_comm::PointCloud& msg)
{
    g_pComm->GetUploadManager().BeginUpload(msg.cloud);
}

void OnGetCloudNextCmd(const eagle_comm::GsCmdSimple& msg)
{
    g_pComm->GetUploadManager().Unlock();
}

void OnGetCloudEndCmd(const eagle_comm::GsCmdSimple& msg)
{
    g_pComm->GetUploadManager().EndUpload();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eagle_comm");

    ros::NodeHandle nh("~");
    int rate = nh.param("rate", COMM_RATE_DEF);

    ros::Subscriber droneStateSub = nh.subscribe("out/drone_state", 1, OnDroneStateUpdated);
    ros::Subscriber pointCloudSub = nh.subscribe("out/point_cloud", 1, OnPointCloudReceived);
    ros::Subscriber getCloudNextSub = nh.subscribe("in/cmd_get_cloud_next", 1, OnGetCloudNextCmd);
    ros::Subscriber getCloudEndSub = nh.subscribe("in/cmd_get_cloud_end", 1, OnGetCloudEndCmd);

    ros::Rate loopRate(rate);

    while (ros::ok())
    {
        g_pComm->Update();

        ros::spinOnce();
        loopRate.sleep();
    }

    CCommunicator::Destroy();

    return 0;
}
