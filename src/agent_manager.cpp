#include "agent_manager.hpp"

namespace DynamicPlanning {
    AgentManager::AgentManager(const ros::NodeHandle &nh, const Param &_param, const Mission &_mission, int agent_id)
            : param(_param), mission(_mission) {
        // Initialize agent
        agent = mission.agents[agent_id];
        agent.current_state.position = mission.agents[agent_id].start_position;

        // Initialize planner states
        planner_state = PlannerState::WAIT;

        // Initialize flags
        has_current_state = false;
        has_obstacles = false;
        has_local_map = false;
        is_disturbed = false;

        //Trajectory Planner
        traj_planner = std::make_unique<TrajPlanner>(nh, param, mission, agent_id);

        //Map manager
        map_manager = std::make_unique<MapManager>(nh, param, mission, agent_id);

        // Command publisher
        if (param.multisim_experiment) {
            cmd_publisher = std::make_unique<CmdPublisher>(nh, param, mission, agent_id);
        }
    }

    void AgentManager::doStep(double time_step) {
        bool do_step_ideal;
        if (param.multisim_experiment) {
            // Check whether the agent is disturbed
            is_disturbed = cmd_publisher->isDisturbed();
            if (is_disturbed) {
                ROS_INFO_STREAM("[AgentManager] agent " << agent.id << " is disturbed");
                agent.current_state.position = cmd_publisher->getObservedPosition();
                agent.current_state.velocity = point3d(0, 0, 0);
                agent.current_state.acceleration = point3d(0, 0, 0);
                do_step_ideal = false;
            } else {
                do_step_ideal = true;
            }
        } else {
            do_step_ideal = true;
        }

        if (do_step_ideal) {
            // Update agent's current state with ideal future state
            dynamic_msgs::State ideal_future_state = getFutureStateMsg(time_step);
            agent.current_state.position = pointMsgToPoint3d(ideal_future_state.pose.position);
            agent.current_state.velocity = vector3MsgToPoint3d(ideal_future_state.velocity.linear);
            agent.current_state.acceleration = vector3MsgToPoint3d(ideal_future_state.acceleration.linear);
        }

        if (param.world_dimension == 2) {
            agent.current_state.position.z() = (float) param.world_z_2d;
        }

        // update local map
        if (not param.world_use_global_map) {
            map_manager->updateLocalMap(agent.current_state.position);
        }

        has_current_state = true;
    }

    PlanningReport AgentManager::plan(ros::Time sim_current_time) {
        // Input check
        if (!has_obstacles || !has_current_state) {
            return PlanningReport::WAITFORROSMSG;
        }

        if (param.multisim_experiment and planner_state == PlannerState::LAND) {
            cmd_publisher->landingCallback();
        } else {
            // change desired goal position by the agent's state
            planningStateTransition();

            if(is_disturbed){
                ROS_WARN_STREAM("[AgentManager] agent " << agent.id << " disturbance detected");
            }

            // Start planning
            desired_traj = traj_planner->plan(agent,
                                              map_manager->getOctomap(),
                                              map_manager->getDistmap(),
                                              sim_current_time,
                                              is_disturbed);

            // Update trajectory in Command publisher
            if(param.multisim_experiment){
                cmd_publisher->updateTraj(desired_traj, sim_current_time);
            }
        }

        // Re-initialization for replanning
        has_obstacles = false;
        has_current_state = false;

        return PlanningReport::SUCCESS;
    }

    void AgentManager::publish() {
        traj_planner->publish();
        map_manager->publish();
    }

    void AgentManager::obstacleCallback(const dynamic_msgs::ObstacleArray &msg_obstacles) {
        traj_planner->setObstacles(msg_obstacles);
        has_obstacles = true;
    }

    void AgentManager::mergeMapCallback(const octomap_msgs::Octomap &msg_merge_map) {
        map_manager->mergeMapCallback(msg_merge_map);
    }

    bool AgentManager::isInitialStateValid() {
        bool is_valid;
        point3d observed_position;
        if (param.multisim_experiment and cmd_publisher->externalPoseUpdate()) {
            observed_position = cmd_publisher->getObservedPosition();
            double dist = observed_position.distance(agent.current_state.position);
            is_valid = dist < param.reset_threshold;
            if (not is_valid) {
                ROS_WARN_STREAM(
                        "observed: " << observed_position << ", ideal: " << agent.current_state.position << ", dist:"
                                     << dist);
            }
        } else {
            is_valid = true; // Use ideal state instead -> valid
        }

        return is_valid;
    }

    void AgentManager::setCurrentState(const dynamic_msgs::State &msg_current_state) {
        // update agent.current_state
        agent.current_state.position = pointMsgToPoint3d(msg_current_state.pose.position);
        if (param.world_dimension == 2) {
            agent.current_state.position.z() = (float) param.world_z_2d;
        }
        agent.current_state.velocity = vector3MsgToPoint3d(msg_current_state.velocity.linear);
        agent.current_state.acceleration = vector3MsgToPoint3d(msg_current_state.acceleration.linear);

        // update flag
        has_current_state = true;
    }

    void AgentManager::setPlannerState(const PlannerState &new_planner_state) {
        if(param.multisim_experiment and planner_state == PlannerState::LAND and not cmd_publisher->landingFinished()){
            return;
        }

        planner_state = new_planner_state;
    }

    void AgentManager::setStartPosition(const point3d &_start_position) {
        mission.agents[agent.id].start_position = _start_position;
        agent.start_position = _start_position;
    }

    void AgentManager::setDesiredGoal(const point3d &new_desired_goal_position) {
        mission.agents[agent.id].desired_goal_position = new_desired_goal_position;
        agent.desired_goal_position = new_desired_goal_position;
    }

    void AgentManager::setGlobalMap(const sensor_msgs::PointCloud2 &global_map) {
        map_manager->setGlobalMap(global_map);
    }

    point3d AgentManager::getCurrentPosition() const {
        return agent.current_state.position;
    }

    dynamic_msgs::State AgentManager::getCurrentStateMsg() const {
        dynamic_msgs::State msg_current_state;
        msg_current_state.pose.position = point3DToPointMsg(agent.current_state.position);
        msg_current_state.velocity = point3DToTwistMsg(agent.current_state.velocity);
        msg_current_state.acceleration = point3DToTwistMsg(agent.current_state.acceleration);
        return msg_current_state;
    }

    dynamic_msgs::State AgentManager::getFutureStateMsg(double future_time) const {
        dynamic_msgs::State msg_current_state = getStateFromControlPoints(desired_traj, future_time,
                                                                          param.M, param.n, param.dt);
        return msg_current_state;
    }

    PlanningStatistics AgentManager::getPlanningStatistics() const {
        return traj_planner->getPlanningStatistics();
    }

    traj_t AgentManager::getTraj() const {
        return desired_traj;
    }

    int AgentManager::getPlannerSeq() const {
        return traj_planner->getPlannerSeq();
    }

    point3d AgentManager::getCurrentGoalPosition() const {
        return traj_planner->getCurrentGoalPosition();
    }

    point3d AgentManager::getDesiredGoalPosition() const {
        return agent.desired_goal_position;
    }

    dynamic_msgs::Obstacle AgentManager::getObstacleMsg() const {
        dynamic_msgs::Obstacle msg_obstacle;
        msg_obstacle.id = agent.id;
        msg_obstacle.type = ObstacleType::AGENT;
        msg_obstacle.pose.position = point3DToPointMsg(agent.current_state.position);
        msg_obstacle.pose.orientation = defaultQuaternion();
        msg_obstacle.velocity = point3DToTwistMsg(agent.current_state.velocity);
        msg_obstacle.goal = point3DToPointMsg(agent.desired_goal_position);
        msg_obstacle.radius = (float) agent.radius;
        msg_obstacle.downwash = (float) agent.downwash;
        msg_obstacle.max_acc = (float) agent.max_acc[0];
        if (not desired_traj.empty()) {
            msg_obstacle.prev_traj = trajToTrajMsg(desired_traj, agent.id, param.dt);
        }
        return msg_obstacle;
    }

    octomap_msgs::Octomap AgentManager::getOctomapMsg() const {
        return map_manager->getOctomapMsg();
    }

    void AgentManager::planningStateTransition() {
        if (planner_state == PlannerState::GOTO) {
            agent.desired_goal_position = mission.agents[agent.id].desired_goal_position;
        } else if (planner_state == PlannerState::PATROL and
                   agent.desired_goal_position.distance(agent.current_state.position) < param.goal_threshold) {
            // Swap start and goal position
            point3d temp = agent.desired_goal_position;
            agent.desired_goal_position = agent.start_position;
            agent.start_position = temp;
        } else if (planner_state == PlannerState::GOBACK) {
            // Go back to start position
            agent.desired_goal_position = mission.agents[agent.id].start_position;
        }

        // if planner_state == PlannerState::WAIT, use previous desired goal position
    }
}