///@file station_test_node.cpp
///@author Vinson Sheep (775014077@qq.com)
///@brief 用于发布指令
///@version 1.0
///@date 2021-10-15
///
///@copyright Copyright (c) 2021
///
#include "ros/ros.h"
#include "radio_proxy/Command.h"
#include "string"

ros::Publisher cmdPub;
int uav_num;
radio_proxy::Command cmd;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"station_test_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("uav_num", uav_num, 1);

    // initialize command publisher
    cmdPub = nh.advertise<radio_proxy::Command>("/station_proxy/command",10);

    cmd.target_id = 255;
    cmd.header.frame_id = "station";

    // main loop
    while(ros::ok()){

        // take off

        std::string s;

        std::cout << "Pless any key to takeoff:" << std::endl;
        std::cin >> s;

        cmd.header.stamp = ros::Time::now();
        cmd.command = radio_proxy::Command::TAKEOFF;
        cmdPub.publish(cmd);
        ros::spinOnce();

        // take off end

        // move

        while (ros::ok()){
            float x, y, z, yaw;

            std::cout << "Pless move to: (x, y, z, yaw) (input 0 0 0 0 for stop)" << std::endl;
            std::cin >> x >> y >> z >> yaw;

            if (std::abs(x) < 10e-5 && std::abs(y) < 10e-5 && std::abs(z) < 10e-5){
                break;
            }

            cmd.header.stamp = ros::Time::now();
            cmd.command = radio_proxy::Command::SETPOINT_LOCAL;
            cmd.x = x;
            cmd.y = y;
            cmd.z = z;
            cmd.yaw = yaw;

            cmdPub.publish(cmd);

            ros::spinOnce();
        }

        // move end 

        // land
        
        std::cout << "Pless any key to land:" << std::endl;
        std::cin >> s;

        cmd.header.stamp = ros::Time::now();
        cmd.command = radio_proxy::Command::LAND;

        cmdPub.publish(cmd);

        ros::spinOnce();

        // land end

    }

    return 0;
}