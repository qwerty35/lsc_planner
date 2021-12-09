#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <sp_const.hpp>
#include <string>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

namespace DynamicPlanning{
    class Param {
    public:
        bool log;
        std::string package_path;
        std::vector<std::string> mission_file_names;

        // World
        std::string world_frame_id;
        std::vector<std::string> world_file_names;
        int world_dimension;
        bool world_use_octomap;
        double world_resolution;
        double world_z_2d;

        // Multisim setting
        int multisim_planning_rate;
        int multisim_qn;
        double multisim_time_step;
        bool multisim_patrol;
        double multisim_max_noise;
        int multisim_max_planner_iteration;
        bool multisim_save_result;
        bool multisim_replay;
        std::string multisim_replay_file_name;
        bool multisim_experiment;
        double multisim_record_time_step;
        int multisim_scene_iteration;
        double multisim_reset_threshold;

        // Planner mode
        PlannerMode planner_mode;
        PredictionMode prediction_mode;
        InitialTrajMode initial_traj_mode;
        SlackMode slack_mode;
        GoalMode goal_mode;

        // Obstacle prediction
        bool obs_size_prediction;
        double obs_uncertainty_horizon;
        bool obs_agent_clustering;

        // Trajectory representation
        double dt;
        int M; // the number of polynomial segment
        int n; // degree of polynomial
        int phi; // desired derivatives e.g. 3 -> minimize jerk
        int phi_n; // not used now, fix it to 1

        // Trajectory optimization
        double control_input_weight;
        double terminal_weight;
        double slack_collision_weight;
        int N_constraint_segments; // for bso, bvc

        // Deadlock
        double deadlock_velocity_threshold;
        int deadlock_seq_threshold;

        // Filter
        double filter_sigma_y_sq;
        double filter_sigma_v_sq;
        double filter_sigma_a_sq;

        // ORCA
        double orca_horizon;
        double orca_inflation_ratio;
        double ocra_pref_velocity_ratio;

        // Grid-based planner
        double grid_resolution;
        double grid_margin;

        // Goal
        double goal_threshold;
        double goal_radius;
        double priority_dist_threshold;

        // Communication
        double communication_range;

        // Exploration
        double sensor_range;


        bool initialize(const ros::NodeHandle &nh);
        [[nodiscard]] std::string getPlannerModeStr() const;
        [[nodiscard]] std::string getPredictionModeStr() const;
        [[nodiscard]] std::string getInitialTrajModeStr() const;
        [[nodiscard]] std::string getSlackModeStr() const;
        [[nodiscard]] std::string getGoalModeStr() const;
    };
}
