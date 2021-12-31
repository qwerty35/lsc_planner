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
#include <sensor_msgs/PointCloud2.h>

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
        TrajPlanner(const ros::NodeHandle& nh, const Param& param, const Mission& mission, int agent_id);

        traj_t plan(const Agent& agent,
                    const std::shared_ptr<octomap::OcTree>& octree_ptr,
                    const std::shared_ptr<DynamicEDTOctomap>& distmap_ptr,
                    ros::Time sim_current_time,
                    bool is_disturbed);

        void publish();

//        void reset(const dynamic_msgs::State& msg_current_state);

        // Setter
        void setObstacles(const dynamic_msgs::ObstacleArray &msg_obstacles);

        // Getter
        [[nodiscard]] int getPlannerSeq() const;

        [[nodiscard]] point3d getCurrentGoalPosition() const;

        [[nodiscard]] point3d getAgentORCAVelocity() const;

        [[nodiscard]] point3d getObsORCAVelocity(int oi) const;

        [[nodiscard]] PlanningStatistics getPlanningStatistics() const;

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

        // Agent state, report
        Agent agent;
        int planner_seq;
        PlanningStatistics statistics;
        bool initialize_sfc, is_disturbed;
        GoalPlannerState goal_planner_state;

        // Frequently used constants
        int M, n;
        Eigen::MatrixXd B, B_inv;

        // Trajectories
        traj_t initial_traj; // [segment_idx][control_pts_idx], initial trajectory
        traj_t prev_traj; // [segment_idx][control_pts_idx], previous trajectory

        // Obstacle
        std::shared_ptr<octomap::OcTree> octree_ptr; // octomap
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr; // Euclidean distance field map
        std::vector<Obstacle> obstacles; // [obs_idx], obstacles
        std::vector<traj_t> obs_pred_trajs; // [obs_idx][segment_idx][control_pts_idx], predicted trajectory of obstacles
        std::vector<std::vector<std::vector<double>>> obs_pred_sizes; // [obs_idx][segment_idx][control_pts_idx], predicted obstacle size
        std::set<int> obs_slack_indices; // set of obstacle indices that need to adjust slack variable, needed for dealing with external disturbance

        // Goal planning
        points_t grid_path;
        int prior_obs_id;

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

        // ROS
        void initializeROS(int agent_id);

        // Planner module
        traj_t planImpl();
        traj_t planORCA();
        traj_t planLSC();

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
        void goalPlanningWithPriority4();
        void goalPlanningWithEntropy();

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

        static point3d normalVector(const point3d& obs_start, const point3d& obs_goal,
                                    const point3d& agent_start, const point3d& agent_goal,
                                    double& closest_dist);

        static point3d normalVectorBetweenPolys(const points_t& control_points_agent,
                                                const points_t& control_points_obs);

        // Trajectory Optimization
        traj_t trajOptimization();

        // Publish
        void publishCollisionConstraints();
        void publishInitialTraj();
        void publishGridPath();
        void publishObstaclePrediction();

        // Utility functions
        [[nodiscard]] int findObstacleIdxByObsId(int obs_id) const;
        [[nodiscard]] double distanceToGoalByObsId(int obs_id) const;
        [[nodiscard]] double distanceToGoalByObsIdx(int obs_idx) const;

        double computeCollisionTimeToDistmap(const point3d& start_position,
                                             const point3d& goal_position,
                                             double agent_radius,
                                             double time_horizon);

        double computeMinCollisionTime();

        point3d findProperGoalByDirection(const point3d& start, const vector3d& direction, double dist_keep);

        double computeEntropy(const point3d& search_point);
    };
}