#ifndef LSC_PLANNER_CMD_PUBLISHER_H
#define LSC_PLANNER_CMD_PUBLISHER_H

#include <ros/ros.h>
#include <mission.hpp>
#include <util.hpp>
#include <dynamic_msgs/TrajectoryArray.h>
#include <dynamic_msgs/FullState.h>
#include <dynamic_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

namespace DynamicPlanning {
    class CmdPublisher {
    public:
        CmdPublisher(const ros::NodeHandle& _nh, Mission _mission);
        std::vector<ros::Publisher> pubs_cmd;
        ros::Publisher pub_cmd_vis;
        ros::Subscriber sub_desired_trajs;
        ros::ServiceServer service_stop_planning;

        void run();

    private:
        ros::NodeHandle nh;
        Mission mission;

        size_t M, n;
        double dt, landing_time;

        size_t latest_planner_seq;
        std::queue<std::vector<traj_t>> trajs_queue;
        std::queue<ros::Time> trajs_start_time_queue;
        std::vector<traj_t> current_traj;
        ros::Time current_traj_start_time, stop_planning_time;
        bool stop_planning;

        void trajsCallback(const dynamic_msgs::TrajectoryArray& msg_trajs);
        bool stopPlanningCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
        void updateTraj();
        void publishCommand();
    };
}

#endif //LSC_PLANNER_CMD_PUBLISHER_H
