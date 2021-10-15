///@file station_console_node.cpp
///@author Vinson Sheep (775014077@qq.com)
///@brief  用于订阅和显示无人机的状态信息(最高支持8台无人机)
///@version 0.1
///@date 2021-10-15
///
///@copyright Copyright (c) 2021
///
#include "ros/ros.h"
#include "radio_proxy/FlightData.h"
#include "radio_proxy/Status.h"
#include "std_msgs/String.h"
#include "boost/bind.hpp"
#include "sstream"

int uav_num;

radio_proxy::FlightData fd[9];
radio_proxy::Status st[9];

ros::Subscriber fdSub[9];
ros::Subscriber stSub[9];
ros::Subscriber msgSub[9];

ros::Timer mainloopTimer;

void fdCB(const radio_proxy::FlightData::ConstPtr& msg_p, int index){
    fd[index] = *msg_p;
    
}

void stCB(const radio_proxy::Status::ConstPtr& msg_p, int index){
    st[index] = *msg_p;
}

void msgCB(const std_msgs::String::ConstPtr& msg_p, int index){
    std::stringstream ss;
    ss << "dji_" << index << ": " << msg_p->data;
    ROS_INFO("%s", ss.str().c_str());
}

///@brief print data in constant interval
///
///@param event 
void mainLoopCB(const ros::TimerEvent &event){
    std::stringstream ss;

    for (int i=1; i<=uav_num; i++){
        ss << std::endl << "dji_" << i << ": ( ";
        ss << "latitude: " << fd[i].latitude << ", longitude: " << fd[i].longitude << ", altitude: " << fd[i].altitude;
        ss << ", x: " << fd[i].x << ", y: " << fd[i].y << ", z: " << fd[i].z << ", yaw: " << fd[i].yaw << ", height: " << fd[i].height_above_takeoff;
        ss << ", battery_v: " << st[i].battery_v <<  ", gps_health: " << int(st[i].gps_health);
        ss << ")";
    }

    ROS_INFO("%s", ss.str().c_str());

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"station_test_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("uav_num", uav_num, 1);

    // initialze subscriber
    for (int i=1; i<= uav_num; i++){
        std::stringstream s_fd;
        std::stringstream s_st;
        std::stringstream s_msg;

        s_fd << "/dji_" << i << "/flight_data";
        s_st << "/dji_" << i << "/status";
        s_msg << "/dji_" << i << "/message";

        fdSub[i] = nh.subscribe<radio_proxy::FlightData>(s_fd.str().c_str(), 1, boost::bind(&fdCB, _1, i));
        stSub[i] = nh.subscribe<radio_proxy::Status>(s_st.str().c_str(), 1, boost::bind(&stCB, _1, i));
        msgSub[i] = nh.subscribe<std_msgs::String>(s_msg.str().c_str(), 1, boost::bind(&msgCB, _1, i));
    }

    // initialze timer
    mainloopTimer = nh.createTimer(ros::Duration(1), &mainLoopCB);

    ros::spin();

    return 0;
}