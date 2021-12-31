#ifndef LSC_PLANNER_AGENT_MANAGER_H
#define LSC_PLANNER_AGENT_MANAGER_H

#include <traj_planner.hpp>
#include <map_manager.hpp>
#include <tf/transform_listener.h>

namespace DynamicPlanning {
    class AgentManager {
    public:
        AgentManager(const ros::NodeHandle &nh, const Param &param, const Mission &mission, int agent_id);

        void doStep(double time_step);

        PlanningReport plan(ros::Time sim_current_time);

        void publish();

        void obstacleCallback(const dynamic_msgs::ObstacleArray& msg_obstacles);

        void mergeMapCallback(const octomap_msgs::Octomap& msg_merge_map);

        bool isInitialStateValid();

        // Setter
        void setCurrentState(const dynamic_msgs::State& msg_current_state);

        void setPlannerState(const PlannerState& new_planner_state);

        void setStartPosition(const point3d& new_start_position);

        void setDesiredGoal(const point3d& new_desired_goal);

        void setGlobalMap(const sensor_msgs::PointCloud2& global_map);

        // Getter
        [[nodiscard]] point3d getCurrentPosition() const;

        [[nodiscard]] dynamic_msgs::State getCurrentStateMsg() const;

        [[nodiscard]] dynamic_msgs::State getFutureStateMsg(double future_time) const;

        [[nodiscard]] PlanningStatistics getPlanningStatistics() const;

        [[nodiscard]] traj_t getTraj() const;

        [[nodiscard]] int getPlannerSeq() const;

        [[nodiscard]] point3d getCurrentGoalPosition() const;

        [[nodiscard]] point3d getDesiredGoalPosition() const;

        [[nodiscard]] dynamic_msgs::Obstacle getObstacleMsg() const;

        [[nodiscard]] octomap_msgs::Octomap getOctomapMsg() const;

    private:
        Param param;
        Mission mission;
        tf::TransformListener tf_listener;

        // Flags, states
        PlannerState planner_state;
        bool has_current_state, has_obstacles, has_local_map, is_disturbed;

        // Agent
        Agent agent;
        traj_t desired_traj;

        //Traj Planner
        std::unique_ptr<TrajPlanner> traj_planner;
        std::unique_ptr<MapManager> map_manager;

        void stateTransition();

        bool observeCurrentPosition(point3d& observed_position);
    };
}

#endif //LSC_PLANNER_AGENT_MANAGER_H
