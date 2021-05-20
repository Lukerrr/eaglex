#include "framework/Communicator.h"
#include "framework/Log.h"

#include "ros/ros.h"
#include "eagle_comm/DroneState.h"
#include "eagle_comm/PointCloud.h"

#include <sstream>

#define COMM_PORT_DEF 54000
#define COMM_MAX_LEN_DEF 16384

static CCommunicator* s_pComm = NULL;

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

    s_pComm->Send(state);
}

void OnPointCloudReceived(const eagle_comm::PointCloud& msg)
{
    SPointCloud cloud;
    cloud.cloudSize = msg.cloud.size();
    if(cloud.cloudSize > cloud.cloudMaxSize)
    {
        cloud.cloudSize = cloud.cloudMaxSize;
    }

    for(int i = 0; i < cloud.cloudSize; ++i)
    {
        cloud.cloud[i].x = msg.cloud[i].x;
        cloud.cloud[i].y = msg.cloud[i].y;
        cloud.cloud[i].z = msg.cloud[i].y;
    }

    s_pComm->Send(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eagle_comm");
    ros::NodeHandle nh("~");

    int port = nh.param("port", COMM_PORT_DEF);
    int maxDataLen = nh.param("max_data_len", COMM_MAX_LEN_DEF);
    s_pComm = new CCommunicator(port, maxDataLen);

    if(s_pComm != NULL)
    {
        ros::Subscriber droneStateSub = nh.subscribe("out/drone_state", 1, OnDroneStateUpdated);
        ros::Subscriber pointCloudSub = nh.subscribe("out/point_cloud", 25, OnPointCloudReceived);

        ros::Rate loopRate(10);

        while (ros::ok())
        {
            s_pComm->Update();

            ros::spinOnce();
            loopRate.sleep();
        }

        delete s_pComm;
    }
    else
    {
        ROS_ERROR_NAMED(LOG_NAME, "Cannot allocate the communicator, shutting down...");
    }

    return 0;
}
