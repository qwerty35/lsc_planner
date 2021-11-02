#pragma once
#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <polynomial.hpp>
#include <timer.hpp>
#include <obstacle.hpp>
#include <obstacle_generator.hpp>
#include <kalman_filter.hpp>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_msgs/ObstacleArray.h>
#include <dynamic_msgs/TrajectoryArray.h>
#include <dynamic_msgs/CollisionConstraint.h>
#include <dynamic_msgs/PlanningReport.h>

// Safe Corridor
#include <collision_constraints.hpp>

// Trajectory Optimizer
#include <traj_optimizer.hpp>

//Grid-based planner
#include <grid_based_planner.hpp>

//RVO2-2D
#include <RVO2/RVOSimulator.h>
#include <RVO2/RVO.h>

//RVO2-3D
#include <RVO2-3D/RVOSimulator.h>
#include <RVO2-3D/RVO.h>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <corridor_constructor.hpp>


namespace DynamicPlanning {
    class TrajPlanner {
    public:
        TrajPlanner(int _agent_id, const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission);

        void initializeROS();

        PlanningReport plan(ros::Time _sim_current_time);

        void publish();

        // Setter
        void setCurrentState(const dynamic_msgs::State& msg_current_state);

        void setDistMap(const std::shared_ptr<DynamicEDTOctomap>& distmap_obj);

        void setObstacles(const dynamic_msgs::ObstacleArray& msg_dynamic_obstacles);

        void setObsPrevTrajs(const std::vector<traj_t>& obs_prev_trajs);

        void reset(const dynamic_msgs::State& msg_current_state);

        void updatePlannerState(const PlannerState& new_planner_state);

        void setStart(const octomap::point3d& new_start_position);

        void setDesiredGoal(const octomap::point3d& new_desired_goal);

        // Getter
        octomap::point3d getCurrentPosition() const;

        dynamic_msgs::State getCurrentStateMsg() const;

        dynamic_msgs::State getFutureStateMsg(double future_time) const;

        octomap::point3d getAgentORCAVelocity() const;

        octomap::point3d getObsORCAVelocity(int oi) const;

        PlanningTimeStatistics getPlanningTime() const;

        double getQPCost() const;

        PlanningReport getPlanningReport() const;

        std::vector<std::vector<octomap::point3d>> getTraj() const;

        octomap::point3d getNormalVector(int obs_id, int m) const;

        octomap::point3d getCurrentGoalPosition() const;

        octomap::point3d getDesiredGoalPosition() const;

        int getPlannerSeq() const;

    private:
        Param param;
        Mission mission;

        // ROS
        ros::NodeHandle nh;
        ros::Subscriber sub_current_state;
        ros::Subscriber sub_obstacles;
        ros::Publisher pub_collision_constraints_raw;
        ros::Publisher pub_collision_constraints_vis;
        ros::Publisher pub_initial_traj_raw;
        ros::Publisher pub_initial_traj_vis;
        ros::Publisher pub_obs_pred_traj_raw;
        ros::Publisher pub_obs_pred_traj_vis;
        ros::Publisher pub_grid_path;

        // Times
        ros::Time obstacle_start_time;
        ros::Time obstacle_update_time;
        ros::Time sim_current_time;

        // Flags, state
        FlagMsg flag_current_state, flag_obstacles;
        bool flag_initialize_sfc;
        PlannerState planner_state;

        // Agent
        Agent agent;

        // Report
        PlanningReport planning_report;
        PlanningTimeStatistics planning_time;
        double current_qp_cost;
        int planner_seq;

        // Frequently used constants
        int M, n, phi, dim, N_obs;
        Eigen::MatrixXd B, B_inv;

        // Trajectories
        traj_t initial_traj; // [segment_idx][control_pts_idx], initial trajectory
        traj_t traj_curr; // [segment_idx][control_pts_idx], trajectory planned in the current step

        // Obstacle
        std::shared_ptr<DynamicEDTOctomap> distmap_obj; // octomap
        //TODO: make obstacle struct
        std::vector<dynamic_msgs::Obstacle> obstacles; // [obs_idx], obstacles
        std::vector<traj_t> obs_pred_trajs; // [obs_idx][segment_idx][control_pts_idx], predicted trajectory of obstacles
        std::vector<traj_t> obs_prev_trajs; // [obs_idx][segment_idx][control_pts_idx], trajectory of obstacles planned at the previous step
        std::vector<std::vector<std::vector<double>>> obs_pred_sizes; // [obs_idx][segment_idx][control_pts_idx], predicted obstacle size
        std::set<int> obs_slack_indices; // set of obstacle indices that need to adjust slack variable, needed for dealing with external disturbance

        // Collision constraints
        CollisionConstraints constraints;

        // Trajectory optimizer
        std::unique_ptr<TrajOptimizer> traj_optimizer;

        // Kalman filter
        std::vector<LinearKalmanFilter> linear_kalman_filters;

        // ORCA
        std::unique_ptr<RVO2D::RVOSimulator> rvo_simulator_2d;
        std::unique_ptr<RVO3D::RVOSimulator> rvo_simulator_3d;
        std::vector<octomap::point3d> orca_velocities;

        // Grid based planner
        std::vector<octomap::point3d> grid_path;
        octomap::point3d grid_los_goal;

        // Callback
        void currentStateCallback(const dynamic_msgs::State& msg_current_state);
        void obstaclesCallback(const dynamic_msgs::ObstacleArray& msg_obstacles);

        // Planner module
        void planImpl();
        void planORCA();
        void planLSC();

        // Functions for checking agent state
        bool isDeadlock() const;
        void checkPlannerMode(); // Check modes in launch file are valid, and fix them automatically

        // Goal planning
        void goalPlanning();
        void goalPlanningWithStaticGoal();
        void goalPlanningWithORCA();
        void goalPlanningWithRightHandRule();
        void goalPlanningWithPriority();
        void goalPlanningWithPriority2();

        // Obstacle prediction
        void obstaclePrediction();
        void obstaclePredictionWithCurrPos(); // Need the position of obstacles
        void obstaclePredictionWithLinearKalmanFilter(); // Need the position of obstacles
        void obstaclePredictionWithCurrVel(); // Need the position and velocity of obstacles
        void obstaclePredictionWithOracle(); // Need true trajectory of obstacles
        void obstaclePredictionWithORCA(); // Need the position and velocity of obstacles
        void obstaclePredictionWithPrevSol(); // Need trajectory of other agents planned in the previous step.
                                              // Dynamic obstacle -> current velocity, Agent -> prev sol
        void obstaclePredictionCheck(); // Check obstacle is at the start point of predicted trajectory
                                        // If not, correct predicted trajectory using obstacle position.
        void obstacleSizePredictionWithConstAcc(); // Predict obstacle size using constant acceleration model

        // Initial trajectory planning
        void initialTrajPlanning();
        void initialTrajPlanningGreedy();
        void initialTrajPlanningPrevSol();
        void initialTrajPlanningORCA();
        void initialTrajPlanningCurrVel();
        void initialTrajPlanningCurrPos();
        void initialTrajPlanningCheck(); // Check agent is at the start point of initial trajectory
                                         // If not, correct predicted trajectory using obstacle position.

        // ORCA
        void updateORCAVelocity(bool isObsPredWithORCA);
        void updateORCAVelocity2D(bool isObsPredWithORCA);
        void updateORCAVelocity3D(bool isObsPredWithORCA);

        // Collision constraints
        void generateCollisionConstraints();
        void generateReciprocalRSFC(); // used in RAL2021 submission
        void generateLSC();
        void generateBVC();
        void generateSFC();
        void generateFeasibleSFC();

        static octomap::point3d normalVector(const octomap::point3d& obs_start, const octomap::point3d& obs_goal,
                                      const octomap::point3d& agent_start, const octomap::point3d& agent_goal,
                                      double& closest_dist);

        static octomap::point3d normalVectorGreedy(const octomap::point3d& obs_start, const octomap::point3d& obs_goal,
                                            const octomap::point3d& agent_start, const octomap::point3d& agent_goal,
                                            double r);

        octomap::point3d normalVectorStaticObsGreedy(const octomap::point3d& obstacle_point,
                                                     const octomap::point3d& start_point,
                                                     const octomap::point3d& initial_goal_point,
                                                     const octomap::point3d& desired_goal_point,
                                                     const octomap::point3d& static_obs_type,
                                                     double radius);

        octomap::point3d normalVectorStaticObs(const dynamic_msgs::Obstacle& obstacle,
                                               const octomap::point3d& pi_i_0,
                                               const octomap::point3d& pi_i_1,
                                               const octomap::point3d& obstacle_point,
                                               const octomap::point3d& static_obs_type);

        static octomap::point3d normalVectorBetweenPolys(const std::vector<octomap::point3d>& control_points_agent,
                                                  const std::vector<octomap::point3d>& control_points_obs);

        // Trajectory Optimization
        bool trajOptimization();

        // Publish
        void publishCollisionConstraints();
        void publishInitialTraj();
        void publishGridPath();
        void publishObstaclePrediction();

        // Utility functions
        int findObstacleIdxById(int obs_id) const;

        // transform cuboid static obstacle to effective sphere obstacle located at param point
        octomap::point3d transformStaticObstacle(int oi, octomap::point3d& static_obs_type);

        double computeCollisionTimeToDistmap(const octomap::point3d& start_position,
                                             const octomap::point3d& goal_position,
                                             double agent_radius,
                                             double time_horizon);

        double computeMinCollisionTime();
    };
}