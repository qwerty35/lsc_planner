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
#include <tf/transform_listener.h>


namespace DynamicPlanning {
    class CmdPublisher {
    public:
        CmdPublisher(const ros::NodeHandle& nh, const Param& param, const Mission& mission, int agent_id);

        void updateTraj(const traj_t& new_traj, const ros::Time& traj_start_time);

        void landingCallback();

        [[nodiscard]] bool isDisturbed() const;

        [[nodiscard]] bool externalPoseUpdate() const;

        [[nodiscard]] bool landingFinished() const;

        [[nodiscard]] point3d getObservedPosition() const;

    private:
        Param param;
        Mission mission;

        ros::NodeHandle nh;
        ros::Timer cmd_timer;
        ros::Publisher pub_cmd;
        ros::Publisher pub_cmd_stop;
        ros::Publisher pub_cmd_vis;
        tf::TransformListener tf_listener;

        int agent_id;
        size_t M, n;
        double dt, landing_time;

        point3d observed_position;
        std::queue<traj_t> traj_queue;
        std::queue<ros::Time> traj_start_time_queue;
        traj_t current_traj;
        ros::Time current_traj_start_time, landing_start_time;
        bool initialized, external_pose_update, is_disturbed, landing;

        void cmdTimerCallback(const ros::TimerEvent& event);
        void observeCurrentPosition();
        void loadCurrentTraj();
        bool computeDesiredState(dynamic_msgs::State& desired_state);
        void detectDisturbance(dynamic_msgs::State& desired_state);
        void publishCommand(const dynamic_msgs::State& desired_state);
        void publishLandingCommand(const dynamic_msgs::State& desired_state);
        void failsafe();
    };
}

#endif //LSC_PLANNER_CMD_PUBLISHER_H
