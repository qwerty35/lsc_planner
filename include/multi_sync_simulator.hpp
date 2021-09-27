#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_msgs/TrajectoryArray.h>
#include <dynamic_msgs/GoalArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <mission.hpp>
#include <obstacle_generator.hpp>
#include <traj_planner.hpp>
#include <utility>
#include <fstream>
#include <istream>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <dynamic_msgs/UpdateGoals.h>

namespace DynamicPlanning {
    class MultiSyncSimulator {
    public:
        MultiSyncSimulator(const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission);

        void setOctomap(std::string file_name);
        void run();

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_agent_trajectories;
        ros::Publisher pub_obstacle_trajectories;
        ros::Publisher pub_collision_model;
        ros::Publisher pub_start_goal_points_vis;
        ros::Publisher pub_goal_positions_raw;
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
        ros::Publisher pub_desired_trajs_raw;
        ros::Publisher pub_desired_trajs_vis;
        ros::Publisher pub_grid_map;
        ros::Subscriber sub_octomap;
        ros::ServiceServer service_start_planning;
        ros::ServiceServer service_start_patrol;
        ros::ServiceServer service_stop_patrol;
        ros::ServiceServer service_update_goal;
        tf::TransformListener tf_listener;

        const Param param;
        Mission mission;
        std::shared_ptr<DynamicEDTOctomap> distmap_obj;
        std::vector<std::unique_ptr<TrajPlanner>> agents;
        ObstacleGenerator obstacle_generator;
        visualization_msgs::MarkerArray msg_agent_trajectories;
        visualization_msgs::MarkerArray msg_obstacle_trajectories;

        PlannerState planner_state;
        std::string file_name_time, file_name_param;
        ros::Time sim_start_time, sim_current_time;
        bool is_collided, has_distmap, initial_update, mission_changed;
        double total_flight_time, total_distance, N_average;
        PlanningTimeStatistics planning_time;
        double safety_ratio_agent, safety_ratio_obs;

        void setCurrentState(int qi, const dynamic_msgs::State& state);

        bool isPlannerReady();

        bool plan();

        void publish();

        bool isFinished();

        void doStep();

        void update();

        void summarizeResult();

        void initializeTimer();

        void setObstacles(int qi, const dynamic_msgs::ObstacleArray& dynamic_obstacles);

        void savePlanningResult();

        void savePlanningResultAsCSV();

        void saveSummarizedResultAsCSV();

        void saveNormalVectorAsCSV();

        double getTotalDistance();

        void octomapCallback(const octomap_msgs::Octomap& octomap_msg);

        bool startPlanningCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool startPatrolCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool stopPatrolCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool updateGoalCallback(dynamic_msgs::UpdateGoals::Request& req, dynamic_msgs::UpdateGoals::Response& res);

        void publishCollisionModel();

        void publishStartGoalPoints();

        void publishWorldBoundary();

        void publishAgentTrajectories();

        void publishCollisionAlert();

        void publishAgentState();

        void publishDesiredTrajs();

        void publishGridMap();
    };
}