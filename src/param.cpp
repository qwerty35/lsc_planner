#include <param.hpp>

namespace DynamicPlanning {
    bool Param::initialize(const ros::NodeHandle &nh) {
        std::string mission_file_name, world_file_name;
        nh.param<std::string>("mission", mission_file_name, "default.json");
        nh.param<bool>("log", log, false);

        // World
        nh.param<std::string>("world/frame_id", world_frame_id, "world");
        nh.param<std::string>("world/file_name", world_file_name, "default.bt");
        nh.param<int>("world/dimension", world_dimension, 3);
        nh.param<bool>("world/use_octomap", world_use_octomap, false);
        nh.param<double>("world/resolution", world_resolution, 0.1);
        nh.param<double>("world/z_2d", world_z_2d, 1.0);
        nh.param<bool>("world/use_global_map", world_use_global_map, true);

        // Multisim setting
        nh.param<int>("multisim/planning_rate", multisim_planning_rate, -1);
        nh.param<int>("multisim/qn", multisim_qn, 2);
        nh.param<double>("multisim/time_step", multisim_time_step, 0.1);
        nh.param<bool>("multisim/patrol", multisim_patrol, false);
        nh.param<double>("multisim/max_noise", multisim_max_noise, 0.0);
        nh.param<int>("multisim/max_planner_iteration", multisim_max_planner_iteration, 1000);
        nh.param<bool>("multisim/save_result", multisim_save_result, false);
        nh.param<bool>("multisim/save_mission", multisim_save_mission, false);
        nh.param<bool>("multisim/replay", multisim_replay, false);
        nh.param<std::string>("multisim/replay_file_name", multisim_replay_file_name, "default.csv");
        nh.param<bool>("multisim/experiment", multisim_experiment, false);
        nh.param<double>("multisim/record_time_step", multisim_save_time_step, 0.1);
        nh.param<double>("multisim/reset_threshold", reset_threshold, 0.1);

        // Planner mode
        std::string planner_mode_str;
        nh.param<std::string>("mode/planner", planner_mode_str, "lsc");
        if(planner_mode_str == "lsc"){
            planner_mode = PlannerMode::LSC;
            prediction_mode = PredictionMode::PREVIOUSSOLUTION;
            initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
            slack_mode = SlackMode::NONE;
        }
        else if(planner_mode_str == "bvc"){
            planner_mode = PlannerMode::BVC;
            prediction_mode = PredictionMode::POSITION;
            initial_traj_mode = InitialTrajMode::POSITION;
            slack_mode = SlackMode::NONE;
        }
        else if(planner_mode_str == "orca"){
            planner_mode = PlannerMode::ORCA;
        }

        // Goal mode
        std::string goal_mode_str;
        nh.param<std::string>("mode/goal", goal_mode_str, "prior_based");
        if(goal_mode_str == "right_hand"){
            goal_mode = GoalMode::RIGHTHAND;
        }
        else if(goal_mode_str == "prior_based"){
            goal_mode = GoalMode::PRIORBASED;
        }
        else if(goal_mode_str == "prior_based2"){
            goal_mode = GoalMode::PRIORBASED2;
        }
        else if(goal_mode_str == "prior_based3"){
            goal_mode = GoalMode::PRIORBASED3;
        }
        else if(goal_mode_str == "prior_based4"){
            goal_mode = GoalMode::PRIORBASED4;
        }
        else if(goal_mode_str == "entropy"){
            goal_mode = GoalMode::ENTROPY;
        }

        // Obstacle prediction
        nh.param<bool>("obs/size_prediction", obs_size_prediction, true);
        nh.param<double>("obs/uncertainty_horizon", obs_uncertainty_horizon, 1);
        nh.param<bool>("obs/agent_clustering", obs_agent_clustering, false);

        // Trajectory representation
        nh.param<double>("traj/dt", dt, 0.2);
        nh.param<int>("traj/M", M, 5);
        nh.param<int>("traj/n", n, 5);
        nh.param<int>("traj/phi", phi, 3);
        nh.param<int>("traj/phi_n", phi_n, 1);

        // Trajectory optimization
        nh.param<double>("opt/control_input_weight", control_input_weight, 1);
        nh.param<double>("opt/terminal_weight", terminal_weight, 1);
        nh.param<double>("opt/slack_collision_weight", slack_collision_weight, 1);
        nh.param<int>("opt/N_constraint_segments", N_constraint_segments, -1);
        if (N_constraint_segments < 0) {
            N_constraint_segments = M;
        }

        // Deadlock
        nh.param<double>("deadlock/velocity_threshold", deadlock_velocity_threshold, 0.1);
        nh.param<int>("deadlock/seq_threshold", deadlock_seq_threshold, 5);

        // Filter
        nh.param<double>("filter/sigma_y_sq", filter_sigma_y_sq, 0.0036);
        nh.param<double>("filter/sigma_v_sq", filter_sigma_v_sq, 0.01);
        nh.param<double>("filter/sigma_a_sq", filter_sigma_a_sq, 1.0);

        // ORCA
        nh.param<double>("orca/horizon", orca_horizon, 2.0);
        nh.param<double>("orca/pref_velocity_ratio", ocra_pref_velocity_ratio, 1.0);
        nh.param<double>("orca/inflation_ratio", orca_inflation_ratio, 1.0);

        // Grid-based planner
        nh.param<double>("grid/resolution", grid_resolution, 0.3);
        nh.param<double>("grid/margin", grid_margin, 0.1);

        // Goal
        nh.param<double>("plan/goal_threshold", goal_threshold, 0.1);
        nh.param<double>("plan/goal_radius", goal_radius, 100.0);
        nh.param<double>("plan/priority_dist_threshold", priority_dist_threshold, 0.4);

        // Communication
        nh.param<double>("communication/range", communication_range, 3.0);

        // Exploration
        nh.param<double>("sensor/range", sensor_range, 3.0);


        package_path = ros::package::getPath("lsc_planner");

        if(mission_file_name.find(".json") != std::string::npos){
            mission_file_names.emplace_back(package_path + "/missions/" + mission_file_name);
        }
        else{
            std::set<std::string> mission_set;
            std::string current_path = package_path + "/missions/" + mission_file_name;
            for(const auto& entry : fs::recursive_directory_iterator(current_path)){
                if(entry.path().string().find(".json") != std::string::npos){
                    mission_set.emplace(entry.path().string());
                }
            }

            for(const auto& file_name : mission_set){
                mission_file_names.emplace_back(file_name);
            }
        }

        if(world_use_octomap){
            if(world_file_name.find(".bt") != std::string::npos){
                world_file_names.emplace_back(package_path + "/world/" + world_file_name);
            }
            else{
                std::set<std::string> world_set;
                std::string current_path = package_path + "/world/" + world_file_name;
                for(const auto& entry : fs::recursive_directory_iterator(current_path)){
                    if(entry.path().string().find(".bt") != std::string::npos){
                        world_set.emplace(entry.path().string());
                    }
                }

                for(const auto& file_name : world_set){
                    world_file_names.emplace_back(file_name);
                }
            }
        }

        return true;
    }

    std::string Param::getPlannerModeStr() const{
        const std::string planner_mode_strs[] = {"LSC", "BVC", "ORCA", "ReciprocalRSFC"};
        return planner_mode_strs[static_cast<int>(planner_mode)];
    }

    std::string Param::getPredictionModeStr() const{
        const std::string prediction_mode_strs[] = {"current_position", "constant_velocity", "linear_kalman_filter",
                                                    "oracle", "orca", "previous_solution"};
        return prediction_mode_strs[static_cast<int>(prediction_mode)];
    }

    std::string Param::getInitialTrajModeStr() const{
        const std::string initial_traj_mode_strs[] = {"greedy", "orca", "current_posiotion", "current_velocity",
                                                      "previous_solution", "skip"};
        return initial_traj_mode_strs[static_cast<int>(initial_traj_mode)];
    }

    std::string Param::getSlackModeStr() const{
        const std::string slack_mode_strs[] = {"none", "dynamical_limit", "collision_constraint"};
        return slack_mode_strs[static_cast<int>(slack_mode)];
    }

    std::string Param::getGoalModeStr() const{
        const std::string planner_mode_strs[] = {"static", "orca", "right_hand",
                                                 "prior_based", "prior_based2",
                                                 "prior_based3", "prior_based4",
                                                 "entropy"};
        return planner_mode_strs[static_cast<int>(goal_mode)];
    }
}