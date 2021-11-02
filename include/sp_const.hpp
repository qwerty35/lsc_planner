#pragma once

#define SP_EPSILON          1e-9
#define SP_EPSILON_FLOAT    1e-5
#define SP_INFINITY         1e+9
#define PI 3.1415

#include <stdexcept>
#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <dynamic_msgs/State.h>
#include <dynamic_msgs/Obstacle.h>

namespace DynamicPlanning {
    typedef std::vector<std::vector<octomap::point3d>> traj_t; // [m][control_pts_idx]

    struct Line {
        Line(const octomap::point3d &start_point_, const octomap::point3d &end_point_) : start_point(start_point_),
                                                                                         end_point(end_point_) {};
        octomap::point3d direction() const {
            if ((end_point - start_point).norm() < SP_EPSILON_FLOAT) {
                return octomap::point3d(0, 0, 0);
            } else {
                return (end_point - start_point).normalized();
            }
        }

        octomap::point3d start_point, end_point;
    };

    typedef std::vector<Line> lines_t;

    enum class PlannerMode {
        LSC,
        BVC,
        ORCA,
        RECIPROCALRSFC
    };

    enum class PredictionMode {
        POSITION,
        VELOCITY,
        LINEARKALMANFILTER,
        ORACLE,
        ORCA,
        PREVIOUSSOLUTION,
    };

    enum class InitialTrajMode {
        GREEDY,
        ORCA,
        POSITION,
        VELOCITY,
        PREVIOUSSOLUTION,
        SKIP,
    };

    enum class SlackMode {
        NONE,
        DYNAMICALLIMIT,
        COLLISIONCONSTRAINT,
    };

    enum class GoalMode {
        STATIC,
        ORCA,
        RIGHTHAND,
        PRIORBASED,
        PRIORBASED2,
    };

    enum PlannerState {
        WAIT,
        GOTO,
        PATROL,
        GOBACK,
    };

    enum PlanningReport {
        Initialized,
        INITTRAJGENERATIONFAILED,
        CONSTRAINTGENERATIONFAILED,
        QPFAILED,
        WAITFORROSMSG,
        SUCCESS,
    };

    struct PlanningTime {
        void update(double time){
            current = time;
            if(time < min){
                min = time;
            }
            if(time > max){
                max = time;
            }

            N_sample++;
            average = (average * (N_sample - 1) + time) / N_sample;
        }

        double current = 0;
        double min = SP_INFINITY;
        double max = 0;
        double average = 0;
        int N_sample = 0;
    };

    struct PlanningTimeStatistics {
        void update(PlanningTimeStatistics new_planning_time){
            initial_traj_planning_time.update(new_planning_time.initial_traj_planning_time.current);
            obstacle_prediction_time.update(new_planning_time.obstacle_prediction_time.current);
            goal_planning_time.update(new_planning_time.goal_planning_time.current);
            lsc_generation_time.update(new_planning_time.lsc_generation_time.current);
            sfc_generation_time.update(new_planning_time.sfc_generation_time.current);
            traj_optimization_time.update(new_planning_time.traj_optimization_time.current);
            total_planning_time.update(new_planning_time.total_planning_time.current);
        }

        PlanningTime initial_traj_planning_time;
        PlanningTime obstacle_prediction_time;
        PlanningTime goal_planning_time;
        PlanningTime lsc_generation_time;
        PlanningTime sfc_generation_time;
        PlanningTime traj_optimization_time;
        PlanningTime total_planning_time;
    };

    enum ObstacleType {
        STATICOBSTACLE,
        DYNAMICOBSTACLE,
        AGENT,
    };

    struct State{
        octomap::point3d position;
        octomap::point3d velocity;
        octomap::point3d acceleration;
    };

    struct FlagMsg{
        bool is_updated = false;
        int planner_seq = -1;
        ros::Time updated_time;
    };

    struct FlagDeadlock{
        bool is_deadlock = false;
        int tracking_obs_id = -1;
    };

    struct Agent{
        int id; // id used in planner
        int cid; // crazyflie id
        State current_state;
        octomap::point3d start_position;
        octomap::point3d desired_goal_position;
        octomap::point3d current_goal_position;
        std::vector<double> max_vel; //TODO: in most case, index 0 is only used
        std::vector<double> max_acc; //TODO: in most case, index 0 is only used
        double radius;
        double downwash;
        double nominal_velocity;
    };

    struct ClosestPoints{
        double dist;
        octomap::point3d closest_point1;
        octomap::point3d closest_point2;
    };
}