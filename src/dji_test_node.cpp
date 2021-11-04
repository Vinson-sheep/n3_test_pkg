#include "ros/ros.h"
#include <string>
#include <sstream>
// dji sdk
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/QuaternionStamped.h"
#include <tf/tf.h>

#include "radio_proxy/Command.h"

// constant
enum Mission {
    IDLE,
    TAKINGOFF,
    TAKEOFFED,
    MOVING,
    LANDING
};

// variable
int id;

int mission = IDLE;


geometry_msgs::PointStamped local_position;
double target_x;
double target_y;
double target_z;
double target_yaw;

// service
ros::ServiceClient     drone_activation_service;
ros::ServiceClient     sdk_ctrl_authority_service;
ros::ServiceClient     drone_task_service;
ros::ServiceClient     set_local_pos_reference;

// subcriber
ros::Subscriber localPosition;
ros::Subscriber cmdSub;

// publisher
ros::Publisher ctrlPosYawPub;

// timer
ros::Timer targetPubTimer;


// bool 返回值由于标志是否处理成功
void cmdCB(const radio_proxy::Command::ConstPtr& msg){

    dji_sdk::Activation activation;
    dji_sdk::SDKControlAuthority sdkAuthority;
    dji_sdk::DroneTaskControl droneTaskControl;
    dji_sdk::SetLocalPosRef localPosReferenceSetter;

    if (msg->target_id != id && msg->target_id != 255){
        return;
    }

    uint8_t cmd = msg->command;

    switch(cmd){

        case radio_proxy::Command::TAKEOFF :

            mission = TAKINGOFF;

            // set local start point

            set_local_pos_reference.call(localPosReferenceSetter);

            // activate
            
            drone_activation_service.call(activation);
            if (!activation.response.result)
            {
                ROS_ERROR("activating failed.");
            }
            
            // get authority
            
            sdkAuthority.request.control_enable = 1;
            sdk_ctrl_authority_service.call(sdkAuthority);
            if (!sdkAuthority.response.result)
            {
                ROS_ERROR("authorizing failed.");
                
            }

            // take off
            
            droneTaskControl.request.task = 4;
            drone_task_service.call(droneTaskControl);
            if (!droneTaskControl.response.result)
            {
                ROS_ERROR("take off failed.");
            }

            mission = TAKEOFFED;

            break;

        case radio_proxy::Command::SETPOINT_LOCAL :

            mission = MOVING;

            target_x = msg->x;
            target_y = msg->y;
            target_z = msg->z;
            target_yaw = msg->yaw;

            break;
        

        case radio_proxy::Command::LAND :

            mission = LANDING;

            droneTaskControl.request.task = 6;
            drone_task_service.call(droneTaskControl);
            if (!droneTaskControl.response.result)
            {
                ROS_ERROR("land failed.");
            }

            mission = IDLE;

            break;
            
    }

}


void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
  local_position = *msg;
}


void targetPubTimerCallback(const ros::TimerEvent& event)
{

    sensor_msgs::Joy controlPosYaw;

    if (mission == MOVING){
        controlPosYaw.axes.push_back(target_x - local_position.point.x);
        controlPosYaw.axes.push_back(target_y - local_position.point.y);

        controlPosYaw.axes.push_back(target_z);
        controlPosYaw.axes.push_back(target_yaw);

        ctrlPosYawPub.publish(controlPosYaw);
    }

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"dji_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("id", id, 1);

    //service
    drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    drone_task_service =nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
    // subsciber
    localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
    cmdSub = nh.subscribe("/dji_proxy/command", 10, &cmdCB);
    // publisher
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    // timer
    targetPubTimer = nh.createTimer(ros::Duration(0.1), targetPubTimerCallback);


    ros::spin();
    return 0;
}
