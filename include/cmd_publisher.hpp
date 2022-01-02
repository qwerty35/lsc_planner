#ifndef LSC_PLANNER_CMD_PUBLISHER_H
#define LSC_PLANNER_CMD_PUBLISHER_H

#include <ros/ros.h>
#include <param.hpp>
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
        CmdPublisher(const ros::NodeHandle& nh, const Param& param, const Mission& mission, int agent_id);

        void updateTraj(const traj_t& new_traj, const ros::Time& traj_start_time, bool is_disturbed);

        void stopPlanningCallback();

    private:
        Param param;
        Mission mission;

        ros::NodeHandle nh;
        ros::Timer cmd_timer;
        ros::Publisher pub_cmd;
        ros::Publisher pub_cmd_vis;

        int agent_id;
        size_t M, n;
        double dt, landing_time;

        std::queue<traj_t> traj_queue;
        std::queue<ros::Time> traj_start_time_queue;
        traj_t current_traj;
        ros::Time current_traj_start_time, stop_planning_time;
        bool is_disturbed, stop_planning;

        void cmdTimerCallback(const ros::TimerEvent& event);
        void trajsCallback(const dynamic_msgs::TrajectoryArray& msg_trajs);
        bool stopPlanningCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
        void loadCurrentTraj();
        void publishCommand();
    };
}

#endif //LSC_PLANNER_CMD_PUBLISHER_H
