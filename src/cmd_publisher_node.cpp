#include "cmd_publisher.hpp"

using namespace DynamicPlanning;

int main(int argc, char* argv[]){
    ROS_INFO("Command publisher for crazyflie");
    ros::init (argc, argv, "cmd_publisher_node");
    ros::NodeHandle nh( "~" );

    std::string file_name;
    nh.param<std::string>("mission", file_name, "default.json");

    Mission mission;
    if (not mission.initialize(file_name, 0, 3)) {
        return -1;
    }

    CmdPublisher cmd_publisher(nh, mission);
    ros::Rate rate(50);
    while (ros::ok()) {
        cmd_publisher.run();
        ros::spinOnce();
        rate.sleep();
    }
}