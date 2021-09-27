#pragma once
#include <ros/ros.h>
#include <param.hpp>
#include <mission.hpp>
#include <obstacle_generator.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <util.hpp>

namespace DynamicPlanning {
    class MultiSyncReplayer {
    public:
        MultiSyncReplayer(const ros::NodeHandle& _nh, Param _param, Mission _mission);

        void initializeReplay();

        void replay(double t);

        void readCSVFile(const std::string& file_name);

        void setOctomap(std::string file_name);

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_agent_trajectories;
        ros::Publisher pub_obstacle_trajectories;
        ros::Publisher pub_collision_model;
        ros::Publisher pub_start_goal_points;
//        ros::Publisher pub_safety_margin_to_agents;
//        ros::Publisher pub_safety_margin_to_obstacles;
        ros::Publisher pub_agent_velocities_x;
        ros::Publisher pub_agent_velocities_y;
        ros::Publisher pub_agent_velocities_z;
        ros::Publisher pub_agent_accelerations_x;
        ros::Publisher pub_agent_accelerations_y;
        ros::Publisher pub_agent_accelerations_z;
        ros::Publisher pub_agent_vel_limits;
        ros::Publisher pub_agent_acc_limits;
        ros::Publisher pub_world_boundary;
        ros::Publisher pub_collision_alert;

        Param param;
        Mission mission;
        std::shared_ptr<DynamicEDTOctomap> distmap_obj;
        ObstacleGenerator obstacle_generator;
        visualization_msgs::MarkerArray msg_agent_trajectories_replay;
        visualization_msgs::MarkerArray msg_obstacle_trajectories_replay;
        std::vector<std::vector<State>> agent_state_history;
        std::vector<std::vector<State>> obstacle_state_history;
//        std_msgs::Float64MultiArray safety_margin_to_agents_replay;
//        std_msgs::Float64MultiArray safety_margin_to_obstacles_replay;
//        std::vector<std::vector<double>> safety_margin_to_agents_history;
//        std::vector<std::vector<double>> safety_margin_to_obstacles_history;

        double timeStep, makeSpan;

        void doReplay(double t);

        void publish_agent_trajectories(double t);

        void publish_obstacle_trajectories(double t);

        void publish_collision_model();

        void publish_start_goal_points();

//        void publish_safety_margin(double t);

        void publish_collision_alert();

        void publish_agent_vel_acc(double t);

        static std::vector<std::string> csv_read_row(std::istream& in, char delimiter);
    };

}
