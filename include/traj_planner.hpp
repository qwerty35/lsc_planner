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


namespace DynamicPlanning {
    class TrajPlanner {
    public:
        TrajPlanner(int _agent_id, const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission,
                    const std::shared_ptr<DynamicEDTOctomap>& _distmap_ptr);

        PlanningReport plan(ros::Time _sim_current_time);

        void publish();

        void reset(const dynamic_msgs::State& msg_current_state);

        // Setter
        void setCurrentState(const dynamic_msgs::State& msg_current_state);

        void setDistMap(const std::shared_ptr<DynamicEDTOctomap>& distmap_ptr);

        void setObstacles(const dynamic_msgs::ObstacleArray& msg_dynamic_obstacles);

        void setObsPrevTrajs(const std::vector<traj_t>& obs_prev_trajs);

        void setPlannerState(const PlannerState& new_planner_state);

        void setStartPosition(const point_t& new_start_position);

        void setDesiredGoal(const point_t& new_desired_goal);

        // Getter
        [[nodiscard]] point_t getCurrentPosition() const;

        [[nodiscard]] dynamic_msgs::State getCurrentStateMsg() const;

        [[nodiscard]] dynamic_msgs::State getFutureStateMsg(double future_time) const;

        [[nodiscard]] point_t getAgentORCAVelocity() const;

        [[nodiscard]] point_t getObsORCAVelocity(int oi) const;

        [[nodiscard]] PlanningTimeStatistics getPlanningTime() const;

        [[nodiscard]] double getTrajCost() const;

        [[nodiscard]] PlanningReport getPlanningReport() const;

        [[nodiscard]] traj_t getTraj() const;

        [[nodiscard]] vector_t getNormalVector(int obs_id, int m) const;

        [[nodiscard]] point_t getCurrentGoalPosition() const;

        [[nodiscard]] point_t getDesiredGoalPosition() const;

        [[nodiscard]] int getPlannerSeq() const;

    private:
        Param param;
        Mission mission;

        // ROS
        ros::NodeHandle nh;
        ros::Publisher pub_collision_constraints_raw;
        ros::Publisher pub_collision_constraints_vis;
        ros::Publisher pub_initial_traj_raw;
        ros::Publisher pub_initial_traj_vis;
        ros::Publisher pub_obs_pred_traj_raw;
        ros::Publisher pub_obs_pred_traj_vis;
        ros::Publisher pub_grid_path;
        ros::Time sim_current_time;

        // Flags, state
        FlagMsg flag_current_state, flag_obstacles;
        bool flag_initialize_sfc;
        PlannerState planner_state;
        int prior_obs_id;

        // Agent
        Agent agent;

        // Report
        PlanningReport planning_report;
        PlanningTimeStatistics planning_time;
        double traj_cost;
        int planner_seq;

        // Frequently used constants
        int M, n, phi, dim;
        Eigen::MatrixXd B, B_inv;

        // Trajectories
        traj_t initial_traj; // [segment_idx][control_pts_idx], initial trajectory
        traj_t desired_traj; // [segment_idx][control_pts_idx], desired trajectory

        // Obstacle
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr; // octomap
        std::vector<Obstacle> obstacles; // [obs_idx], obstacles
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
        points_t orca_velocities;

        // Grid based planner
        points_t grid_path;

        // ROS
        void initializeROS();

        // Planner module
        void planImpl();
        void planORCA();
        void planLSC();

        // Functions for checking agent state
        void checkPlannerMode(); // Check modes in launch file are valid, and fix them automatically
        [[nodiscard]] bool isDeadlock() const;

        // Goal planning
        void goalPlanning();
        void goalPlanningWithStaticGoal();
        void goalPlanningWithORCA();
        void goalPlanningWithRightHandRule();
        void goalPlanningWithPriority();
        void goalPlanningWithPriority2();
        void goalPlanningWithPriority3();

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

        static point_t normalVector(const point_t& obs_start, const point_t& obs_goal,
                                      const point_t& agent_start, const point_t& agent_goal,
                                      double& closest_dist);

        static point_t normalVectorBetweenPolys(const points_t& control_points_agent,
                                                  const points_t& control_points_obs);

        // Trajectory Optimization
        bool trajOptimization();

        // Publish
        void publishCollisionConstraints();
        void publishInitialTraj();
        void publishGridPath();
        void publishObstaclePrediction();

        // Utility functions
        [[nodiscard]] int findObstacleIdxByObsId(int obs_id) const;
        [[nodiscard]] double distanceToGoalByObsId(int obs_id) const;
        [[nodiscard]] double distanceToGoalByObsIdx(int obs_idx) const;

        double computeCollisionTimeToDistmap(const point_t& start_position,
                                             const point_t& goal_position,
                                             double agent_radius,
                                             double time_horizon);

        double computeMinCollisionTime();

        point_t findProperGoalByDirection(const point_t& start, const vector_t& direction, double dist_keep);
    };
}