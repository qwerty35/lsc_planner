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
    typedef octomap::point3d point_t;
    typedef octomap::point3d vector_t;
    typedef std::vector<point_t> points_t;
    typedef std::vector<points_t> traj_t; // [m][control_pts_idx]

    struct Line {
        Line(const point_t &_start_point, const point_t &_end_point)
            : start_point(_start_point), end_point(_end_point) {};

        [[nodiscard]] vector_t direction() const {
            if ((end_point - start_point).norm() < SP_EPSILON_FLOAT) {
                return {0, 0, 0};
            } else {
                return (end_point - start_point).normalized();
            }
        }

        point_t start_point, end_point;
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
        point_t position;
        point_t velocity;
        point_t acceleration;
    };

    struct FlagMsg{
        bool is_updated = false;
        int planner_seq = -1;
        ros::Time updated_time;
    };

    struct Agent{
        int id; // id used in planner
        int cid; // crazyflie id
        State current_state;
        point_t start_position;
        point_t desired_goal_position;
        point_t current_goal_position;
        std::vector<double> max_vel; //TODO: in most case, index 0 is only used
        std::vector<double> max_acc; //TODO: in most case, index 0 is only used
        double radius;
        double downwash;
        double nominal_velocity;
    };

    struct Obstacle{
        ros::Time start_time;
        ros::Time update_time;
        int id;
        ObstacleType type;
        double radius;
        double downwash;
        point_t position;
        vector_t velocity;
        double max_acc;
        point_t goal_position;
    };

    struct ClosestPoints{
        double dist;
        point_t closest_point1;
        point_t closest_point2;
    };
}