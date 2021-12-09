#include <traj_planner.hpp>

namespace DynamicPlanning{
    TrajPlanner::TrajPlanner(int _agent_id, const ros::NodeHandle& _nh,
                             const Param& _param, const Mission& _mission,
                             const std::shared_ptr<DynamicEDTOctomap>& _distmap_ptr)
                             : nh(_nh), param(_param), mission(_mission), distmap_ptr(_distmap_ptr)
    {
        // Initialize agent
        agent = mission.agents[_agent_id];
        agent.current_state.position = mission.agents[_agent_id].start_position;

        // Initialize flags
        flag_current_state.is_updated = false;
        flag_obstacles.is_updated = false;
        flag_initialize_sfc = true;
        prior_obs_id = -1;

        // Initialize trajectory param, offsets
        dim = param.world_dimension;
        M = param.M;
        n = param.n;
        phi = param.phi;
        if(param.N_constraint_segments < 0){
            param.N_constraint_segments = M;
        }

        // Initialize initial, desired trajectory
        initial_traj.resize(M);
        desired_traj.resize(M);
        for(int m = 0; m < M; m++){
            initial_traj[m].resize(n + 1);
            desired_traj[m].resize(n + 1);
        }

        // Initialize planner states
        planner_state = PlannerState::WAIT;
        planner_seq = 0;
        planning_report = PlanningReport::Initialized;
        traj_cost = 0;

        // Initialize trajectory optimization module
        buildBernsteinBasis(n, B, B_inv);
        traj_optimizer = std::make_unique<TrajOptimizer>(param, mission, B);

        // Initialize RVO2
        if(param.world_dimension == 2){
            rvo_simulator_2d = std::make_unique<RVO2D::RVOSimulator>();
            rvo_simulator_2d->setTimeStep(0.5f);
            rvo_simulator_2d->setAgentDefaults(15.0f,10, param.orca_horizon, param.orca_horizon,
                                               agent.radius,
                                               agent.max_vel[0] * param.ocra_pref_velocity_ratio);
        }
        else if(param.world_dimension == 3){
            rvo_simulator_3d = std::make_unique<RVO3D::RVOSimulator>();
            rvo_simulator_3d->setTimeStep(0.5f);
            rvo_simulator_3d->setAgentDefaults(15.0f, 10, param.orca_horizon,
                                               agent.radius,
                                               agent.max_vel[0] * param.ocra_pref_velocity_ratio);
        }
        else{
            throw std::invalid_argument("[TrajPlanner] Invalid simulation dimension");
        }

        // Initialize ROS
        initializeROS();
    }

    PlanningReport TrajPlanner::plan(ros::Time _sim_current_time){
        // Input check
        if (!flag_obstacles.is_updated || !flag_current_state.is_updated ||
            flag_current_state.planner_seq != planner_seq)
        {
            if (planner_seq == 0) {
                planning_report = PlanningReport::Initialized;
            } else {
                planning_report = PlanningReport::WAITFORROSMSG;
            }
            return planning_report;
        }

        // Initialize constraints
        if(planner_seq == 0){
            constraints.initialize(distmap_ptr, obs_slack_indices, param, mission);
        }

        // Update clock
        sim_current_time = _sim_current_time;

        // Start planning
        planner_seq++;
        ros::Time planning_start_time = ros::Time::now();
        planImpl();

        // Re-initialization for replanning
        flag_obstacles.is_updated = false;
        flag_current_state.is_updated = false;
        orca_velocities.clear();
        obs_prev_trajs.clear();

        // Print terminal message
        planning_time.total_planning_time.update((ros::Time::now() - planning_start_time).toSec());
//        ROS_INFO_STREAM("[TrajPlanner] mav" << agent.id
//                                               << ", planning time:" << planning_time.total_planning_time.current
//                                               << ", QP cost:" << current_qp_cost);

        return planning_report;
    }

    void TrajPlanner::publish() {
        // Publish goal, obs prediciton when arguments are all valid
        publishObstaclePrediction();

        // Publish initial trajectory when it is generated
        if (planning_report > PlanningReport::INITTRAJGENERATIONFAILED) {
            publishInitialTraj();
            publishGridPath();
        }

        // Publish collision constraints when they are generated
        if (planning_report > PlanningReport::CONSTRAINTGENERATIONFAILED) {
            publishCollisionConstraints();
        }
    }

    void TrajPlanner::setCurrentState(const dynamic_msgs::State& msg_current_state){
        // update agent.current_state
        if(param.world_dimension == 2){
            agent.current_state.position = point_t(msg_current_state.pose.position.x,
                                                   msg_current_state.pose.position.y,
                                                   param.world_z_2d);
        }
        else{
            agent.current_state.position = pointMsgToPoint3d(msg_current_state.pose.position);
        }
        agent.current_state.velocity = vector3MsgToPoint3d(msg_current_state.velocity.linear);
        agent.current_state.acceleration = vector3MsgToPoint3d(msg_current_state.acceleration.linear);

        // update flag
        flag_current_state.is_updated = true;
        flag_current_state.planner_seq = msg_current_state.planner_seq;
        flag_current_state.updated_time = msg_current_state.header.stamp;
    }

    void TrajPlanner::setDistMap(const std::shared_ptr<DynamicEDTOctomap>& _distmap_ptr){
        distmap_ptr = _distmap_ptr;
    }

    void TrajPlanner::setObstacles(const dynamic_msgs::ObstacleArray& msg_obstacles){
        obstacles.clear();

        size_t N_obs = msg_obstacles.obstacles.size();
        obstacles.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            obstacles[oi].start_time = msg_obstacles.start_time;
            if(obstacles[oi].update_time < msg_obstacles.header.stamp){
                obstacles[oi].update_time = msg_obstacles.header.stamp;
            }

            obstacles[oi].id = msg_obstacles.obstacles[oi].id;
            obstacles[oi].type = static_cast<ObstacleType>(msg_obstacles.obstacles[oi].type);
            obstacles[oi].radius = msg_obstacles.obstacles[oi].radius;
            obstacles[oi].downwash = msg_obstacles.obstacles[oi].downwash;
            obstacles[oi].position = pointMsgToPoint3d(msg_obstacles.obstacles[oi].pose.position);
            obstacles[oi].velocity = vector3MsgToPoint3d(msg_obstacles.obstacles[oi].velocity.linear);
            obstacles[oi].max_acc = msg_obstacles.obstacles[oi].max_acc;
            obstacles[oi].goal_position = pointMsgToPoint3d(msg_obstacles.obstacles[oi].goal);
        }

        flag_obstacles.is_updated = true;
    }

    void TrajPlanner::setObsPrevTrajs(const std::vector<traj_t>& _obs_prev_trajs) {
        obs_prev_trajs = _obs_prev_trajs;
    }

    void TrajPlanner::setStartPosition(const point_t& new_start_position) {
        mission.agents[agent.id].start_position = new_start_position;
        agent.start_position = new_start_position;
    }

    void TrajPlanner::setDesiredGoal(const point_t& new_desired_goal_position) {
        mission.agents[agent.id].desired_goal_position = new_desired_goal_position;
        agent.desired_goal_position = new_desired_goal_position;
    }

    void TrajPlanner::reset(const dynamic_msgs::State& msg_current_state) {
        // Reset current state
        setCurrentState(msg_current_state);

        // Reset current trajectory
        for(int m = 0; m < M; m++){
            for(int i = 0; i < n + 1; i++){
                desired_traj[m][i] = agent.current_state.position;
            }
        }

        // Reset SFC
        constraints.initializeSFC(agent.current_state.position, agent.radius);
    }

    void TrajPlanner::setPlannerState(const PlannerState& new_planner_state){
        planner_state = new_planner_state;
    }

    point_t TrajPlanner::getCurrentPosition() const{
        return agent.current_state.position;
    }

    dynamic_msgs::State TrajPlanner::getCurrentStateMsg() const{
        dynamic_msgs::State msg_current_state;
        msg_current_state.planner_seq = planner_seq;
        msg_current_state.pose.position = point3DToPointMsg(agent.current_state.position);
        msg_current_state.velocity = point3DToTwistMsg(agent.current_state.velocity);
        msg_current_state.acceleration = point3DToTwistMsg(agent.current_state.acceleration);
        return msg_current_state;
    }

    dynamic_msgs::State TrajPlanner::getFutureStateMsg(double future_time) const{
        dynamic_msgs::State msg_current_state = getStateFromControlPoints(desired_traj, future_time,
                                                                          M, n, param.dt);
        msg_current_state.planner_seq = planner_seq;
        return msg_current_state;
    }

    point_t TrajPlanner::getAgentORCAVelocity() const{
        return orca_velocities[0];
    }

    point_t TrajPlanner::getObsORCAVelocity(int oi) const{
        if(obstacles.size() < oi + 2){
            throw std::invalid_argument("[TrajPlanner] orca velocity is not updated yet");
        }

        if(obstacles[oi].type == ObstacleType::AGENT){
            return orca_velocities[oi + 1]; // orca_velocities[0] = agent's orca velocity
        }
        else if(obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE){
            point_t obstacle_velocity = obstacles[oi].velocity;
            if(param.world_dimension == 2){
                obstacle_velocity.z() = 0;
            }
            return obstacle_velocity;
        }
        else{
            throw std::invalid_argument("[TrajPlanner] invalid input of getObsORCAVelocity");
        }
    }

    PlanningTimeStatistics TrajPlanner::getPlanningTime() const{
        return planning_time;
    }

    double TrajPlanner::getTrajCost() const{
        return traj_cost;
    }

    PlanningReport TrajPlanner::getPlanningReport() const{
        return planning_report;
    }

    traj_t TrajPlanner::getTraj() const{
        return desired_traj;
    }

    vector_t TrajPlanner::getNormalVector(int obs_id, int m) const{
        int oi = findObstacleIdxByObsId(obs_id);
        if(oi == -1){
            throw std::invalid_argument("[TrajPlanner] invalid mav_id for normalVector");
        }

        return constraints.getLSC(oi, m, 0).normal_vector;
    }

    point_t TrajPlanner::getCurrentGoalPosition() const{
        return agent.current_goal_position;
    }

    point_t TrajPlanner::getDesiredGoalPosition() const{
        return agent.desired_goal_position;
    }

    int TrajPlanner::getPlannerSeq() const{
        return planner_seq;
    }

    void TrajPlanner::initializeROS() {
        // Initialize ros publisher and subscriber
        std::string prefix = "/mav" + std::to_string(agent.id);
        pub_collision_constraints_raw = nh.advertise<dynamic_msgs::CollisionConstraint>(
                prefix + "_collision_constraints_raw", 1);
        pub_collision_constraints_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "_collision_constraints_vis", 1);
        pub_initial_traj_raw = nh.advertise<dynamic_msgs::TrajectoryArray>(
                prefix + "_initial_traj_raw", 1);
        pub_initial_traj_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "_initial_traj_vis", 1);
        pub_obs_pred_traj_raw = nh.advertise<dynamic_msgs::TrajectoryArray>(
                prefix + "_obs_pred_traj_raw", 1);
        pub_obs_pred_traj_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "_obs_pred_traj_vis", 1);
        pub_grid_path = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "_grid_path_vis", 1);
    }

    void TrajPlanner::planImpl(){
        // Check current planner mode is valid.
        checkPlannerMode();

        // Plan initial trajectory of the other agents
        ros::Time obs_pred_start_time = ros::Time::now();
        obstaclePrediction();
        ros::Time obs_pred_end_time = ros::Time::now();
        planning_time.obstacle_prediction_time.update((obs_pred_end_time - obs_pred_start_time).toSec());

        // Plan initial trajectory of current agent.
        ros::Time init_traj_planning_start_time = ros::Time::now();
        initialTrajPlanning();
        ros::Time init_traj_planning_end_time = ros::Time::now();
        planning_time.initial_traj_planning_time.update((init_traj_planning_end_time - init_traj_planning_start_time).toSec());

        // Goal planning
        ros::Time goal_planning_start_time = ros::Time::now();
        goalPlanning();
        ros::Time goal_planning_end_time = ros::Time::now();
        planning_time.goal_planning_time.update((goal_planning_end_time - goal_planning_start_time).toSec());

        if(param.planner_mode == PlannerMode::ORCA){
            planORCA();
        }
        else{
            // Plan LSC or BVC
            planLSC();
        }
    }

    void TrajPlanner::planORCA(){
        updateORCAVelocity(false);

        point_t new_velocity = getAgentORCAVelocity();
        for(int m = 0; m < M; m++){
            for(int i = 0; i < n + 1; i++){
                double m_intp = m + (double)i/n;
                desired_traj[m][i] = agent.current_state.position + new_velocity * m_intp * param.dt;
            }
        }

        planning_report = PlanningReport::SUCCESS;
    }

    void TrajPlanner::planLSC(){
        // Construct LSC (or BVC) and SFC
        generateCollisionConstraints();

        // Trajectory optimization
        bool success = trajOptimization();
        if (success) {
            planning_report = PlanningReport::SUCCESS;
        }
        else { // if not success
            planning_report = PlanningReport::QPFAILED;
        }
    }

    void TrajPlanner::checkPlannerMode(){
        if(planner_seq > 2){
            return;
        }

        switch(param.planner_mode) {
            case PlannerMode::LSC:
                if(param.multisim_time_step != param.dt){
                    throw std::invalid_argument("[TrajPlanner] multisim_time_step must be equal to segment time");
                }
                if(param.prediction_mode != PredictionMode::PREVIOUSSOLUTION){
                    ROS_WARN("[TrajPlanner] prediction_mode of LSC must be previous_solution, fix to previous_solution");
                    param.prediction_mode = PredictionMode::PREVIOUSSOLUTION;
                }
                if(param.initial_traj_mode != InitialTrajMode::PREVIOUSSOLUTION){
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be previous_solution, fix to previous_solution");
                    param.initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
                }
                if(param.slack_mode != SlackMode::NONE){
                    ROS_WARN("[TrajPlanner] LSC does not need slack variables, fix to none");
                    param.slack_mode = SlackMode::NONE;
                    traj_optimizer->updateParam(param);
                }
                break;
            case PlannerMode::BVC:
                if (param.prediction_mode != PredictionMode::POSITION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of BVC must be current_position, fix to current_position");
                    param.prediction_mode = PredictionMode::POSITION;
                }
                if (param.initial_traj_mode != InitialTrajMode::POSITION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be current_position, fix to current_position");
                    param.initial_traj_mode = InitialTrajMode::POSITION;
                }
                break;
            case PlannerMode::ORCA:
                break;
            case PlannerMode::RECIPROCALRSFC:
                if(param.slack_mode != SlackMode::COLLISIONCONSTRAINT){
                    ROS_WARN("[TrajPlanner] Reciprocal RSFC need slack variables at collision constraints");
                    param.slack_mode = SlackMode::COLLISIONCONSTRAINT;
                    traj_optimizer->updateParam(param);
                }
                break;
        }

        if(param.world_use_octomap and distmap_ptr == nullptr){
            throw std::invalid_argument("[TrajPlanner] distmap is not ready");
        }
    }

    void TrajPlanner::goalPlanning() {
        // Patrol
        if (planner_state == PlannerState::PATROL and
        (agent.desired_goal_position - agent.current_state.position).norm() < param.goal_threshold) {
            // Swap start and goal position
            point_t temp = agent.desired_goal_position;
            agent.desired_goal_position = agent.start_position;
            agent.start_position = temp;
        }
        else if(planner_state == PlannerState::GOBACK){
            // Go back to start position
            agent.desired_goal_position = mission.agents[agent.id].start_position;
            agent.start_position = mission.agents[agent.id].desired_goal_position;
        }

        switch(param.goal_mode){
            case GoalMode::STATIC:
                goalPlanningWithStaticGoal();
                break;
            case GoalMode::ORCA:
                goalPlanningWithORCA();
                break;
            case GoalMode::RIGHTHAND:
                goalPlanningWithRightHandRule();
                break;
            case GoalMode::PRIORBASED:
                goalPlanningWithPriority();
                break;
            case GoalMode::PRIORBASED2:
                goalPlanningWithPriority2();
                break;
            case GoalMode::PRIORBASED3:
                goalPlanningWithPriority3();
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid goal mode");
                break;
        }
    }

    void TrajPlanner::goalPlanningWithStaticGoal() {
        agent.current_goal_position = agent.desired_goal_position;
    }

    void TrajPlanner::goalPlanningWithORCA() {
        updateORCAVelocity(false);
        point_t orca_velocity = getAgentORCAVelocity();
        agent.current_goal_position = agent.current_state.position + orca_velocity * M * param.dt;
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenPointAndLineSegment(agent.desired_goal_position,
                                                                 agent.current_state.position,
                                                                 agent.current_goal_position);
        if(closest_points.dist < 0.05){
            agent.current_goal_position = agent.desired_goal_position;
        }
    }

    void TrajPlanner::goalPlanningWithRightHandRule() {
        // If the agent detect deadlock, then change the goal point to the right.
        if(isDeadlock()){
            point_t z_axis(0, 0, 1);
            agent.current_goal_position = agent.current_state.position +
                                          (agent.desired_goal_position - agent.current_state.position).cross(z_axis);
        }
        else{
            goalPlanningWithStaticGoal();
        }
    }

    void TrajPlanner::goalPlanningWithPriority() {
        // Find higher priority agents
        std::set<int> high_priority_obstacle_ids;
        high_priority_obstacle_ids.clear();
        int closest_obs_id = -1;
        double dist_to_goal = (agent.current_state.position - agent.desired_goal_position).norm();
        double min_dist_to_obs = SP_INFINITY;
        for(int oi = 0; oi < obstacles.size(); oi++){
            if(obs_slack_indices.find(oi) != obs_slack_indices.end()){
                high_priority_obstacle_ids.emplace(obstacles[oi].id);
                continue;
            }

            if(obstacles[oi].type == ObstacleType::AGENT) {
                point_t obs_goal_position = obstacles[oi].goal_position;
                point_t obs_curr_position = obstacles[oi].position;
                double obs_dist_to_goal = (obs_curr_position - obs_goal_position).norm();
                double dist_to_obs = (obs_curr_position - agent.current_state.position).norm();

                // Do not consider the priority when other agent is near goal.
                if(obs_dist_to_goal < param.goal_threshold){
                    continue;
                }
                // Do not consider the agents have the same direction
                if(dist_to_goal > param.goal_threshold and (obs_prev_trajs[oi][M-1][n] - obs_prev_trajs[oi][0][n]).dot(obs_prev_trajs[oi][0][n] - agent.current_state.position) > 0){
                    continue;
                }
                // If the agent is near goal, all other agents have higher priority
                // Else the agents with smaller dist_to_goal have higher priority
                if (dist_to_goal < param.goal_threshold or obs_dist_to_goal < dist_to_goal) {
                    if (dist_to_obs < min_dist_to_obs) {
                        min_dist_to_obs = dist_to_obs;
                        closest_obs_id = oi;
                    }
                    high_priority_obstacle_ids.emplace(obstacles[oi].id);
                }
            }
        }

        // If the distance to a higher priority agent is too short, then move away from that agent.
        double priority_dist_threshold = param.priority_dist_threshold;
        double dist_keep = priority_dist_threshold + 0.1; //TODO: param need to consider radius of agents!
        if(min_dist_to_obs < priority_dist_threshold){
            point_t obs_curr_position = obstacles[closest_obs_id].position;
            agent.current_goal_position = agent.current_state.position -
                                          (obs_curr_position - agent.current_state.position).normalized() * dist_keep;
            return;
        }

        // A* considering priority
        GridBasedPlanner grid_based_planner(distmap_ptr, mission, param);
        grid_path = grid_based_planner.plan(agent.current_state.position, agent.desired_goal_position,
                                            agent.id, agent.radius, agent.downwash,
                                            obstacles, high_priority_obstacle_ids);
        if(grid_path.empty()) {
            // A* without priority
            grid_path = grid_based_planner.plan(agent.current_state.position, agent.desired_goal_position,
                                                agent.id, agent.radius, agent.downwash,
                                                obstacles);
        }

        // Find los-free goal from end of the initial trajectory
        point_t los_free_goal = grid_based_planner.findLOSFreeGoal(initial_traj[M-1][n],
                                                                   agent.desired_goal_position,
                                                                   agent.radius);
        agent.current_goal_position = los_free_goal;
    }

    void TrajPlanner::goalPlanningWithPriority2() {
        // Find higher_priority_agents
        std::set<int> higher_priority_agents; // set of obstacle id
        double agent_dist_to_goal = agent.current_state.position.distance(agent.desired_goal_position);
        for(int oi = 0; oi < obstacles.size(); oi++) {
            if (obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE or
                obs_slack_indices.find(oi) != obs_slack_indices.end()) {
                higher_priority_agents.emplace(obstacles[oi].id);
                continue;
            } else if (obstacles[oi].type == ObstacleType::AGENT) {
                point_t obs_goal_position = obstacles[oi].goal_position;
                point_t obs_curr_position = obstacles[oi].position;
                point_t obs_future_position = obs_pred_trajs[oi][M - 1][n];

                if(obs_curr_position.distance(agent.current_state.position) > 3.0){ //TODO: parameter
                    continue;
                }

                // Impose the higher priority to the agents with smaller dist_to_goal
                // Do not consider the agents that have the same direction.
                // Do not consider the agent that is near goal.
                double obs_dist_to_goal = obs_curr_position.distance(obs_goal_position);
                if (obs_dist_to_goal > param.goal_threshold and
                    (obs_future_position - obs_curr_position).dot(obs_curr_position - agent.current_state.position) < 0 and
                    obs_dist_to_goal < agent_dist_to_goal) {
                    higher_priority_agents.emplace(obstacles[oi].id);
                }
            }
        }

        // A* considering priority
        GridBasedPlanner grid_based_planner(distmap_ptr, mission, param);
        grid_path = grid_based_planner.plan(agent.current_state.position, agent.desired_goal_position,
                                            agent.id, agent.radius, agent.downwash,
                                            obstacles, higher_priority_agents);

        if(grid_path.empty()) {
            // A* without priority
            grid_path = grid_based_planner.plan(agent.current_state.position, agent.desired_goal_position,
                                                agent.id, agent.radius, agent.downwash,
                                                obstacles);
        }

        // Find los-free goal from end of the initial trajectory
        point_t los_free_goal = grid_based_planner.findLOSFreeGoal(initial_traj[M-1][n],
                                                                   agent.desired_goal_position,
                                                                   agent.radius);

        // Check prior obstacle has passed.
        //TODO: ghost
        if(prior_obs_id != -1) {
            int prior_obs_idx = findObstacleIdxByObsId(prior_obs_id);
            point_t prior_obs_future_position = obs_pred_trajs[prior_obs_idx][M - 1][n];
            ClosestPoints closest_points = closestPointsBetweenPointAndLineSegment(prior_obs_future_position,
                                                                                   agent.current_state.position,
                                                                                   los_free_goal);
            double dist = closest_points.dist;
            if (dist > agent.radius + obstacles[prior_obs_idx].radius) {
                prior_obs_id = -1;
            }
        }

        // Find cluster
        std::set<int> cluster; // set of obstacle idx
        std::queue<int> search_queue; // queue of obstacle idx
        point_t agent_future_position = initial_traj[M-1][n];
        int prior_obs_cand_idx = -1;
        double min_dist_to_goal = SP_INFINITY;
        double cluster_margin = 0.1;
        for(int oi = 0; oi < obstacles.size(); oi++) {
            if (obstacles[oi].type == ObstacleType::AGENT) {
                point_t obs_future_position = obs_pred_trajs[oi][M - 1][n];
                double future_dist_to_obs = ellipsoidalDistance(obs_future_position, agent_future_position,
                                                                agent.downwash);
                double safety_distance = obstacles[oi].radius + agent.radius + cluster_margin;

                // If future position among agents are too close then find the highest priority agent
                if (future_dist_to_obs < safety_distance) {
                    search_queue.push(oi);
                    cluster.emplace(oi);

                    // If the agent is near goal, all other agents have higher priority
                    // Else the agents with smaller dist_to_goal have higher priority
                    // Do not consider the priority when other agent is near goal.
                    double obs_dist_to_goal = distanceToGoalByObsIdx(oi);
                    if (obs_dist_to_goal > param.goal_threshold and obs_dist_to_goal < min_dist_to_goal) {
                        prior_obs_cand_idx = oi;
                        min_dist_to_goal = obs_dist_to_goal;
                    }
                }
            }
        }
        while(not search_queue.empty()){
            int search_obs_idx = search_queue.front();
            point_t search_obs_future_position = obs_pred_trajs[search_obs_idx][M - 1][n];
            search_queue.pop();

            for(int oi = 0; oi < obstacles.size(); oi++){
                if(oi == search_obs_idx or cluster.find(oi) != cluster.end()){
                    continue;
                }

                point_t obs_future_position = obs_pred_trajs[oi][M-1][n];
                double future_dist_to_obs = ellipsoidalDistance(obs_future_position, search_obs_future_position,
                                                                obstacles[search_obs_idx].downwash);
                double safety_distance = obstacles[oi].radius + obstacles[search_obs_idx].radius + cluster_margin;

                if(future_dist_to_obs < safety_distance){
                    search_queue.push(oi);
                    cluster.emplace(oi);

                    double obs_dist_to_goal = distanceToGoalByObsIdx(oi);
                    if (obs_dist_to_goal > param.goal_threshold and obs_dist_to_goal < min_dist_to_goal) {
                        prior_obs_cand_idx = oi;
                        min_dist_to_goal = obs_dist_to_goal;
                    }
                }
            }
        }

        // Determine prior agent
        if(prior_obs_id != -1){
            double prior_dist_to_goal = distanceToGoalByObsId(prior_obs_id);
            if(prior_dist_to_goal < min_dist_to_goal){
                min_dist_to_goal = prior_dist_to_goal;
            }
            else if(prior_obs_cand_idx != -1){
                prior_obs_id = obstacles[prior_obs_cand_idx].id;
            }
        }
        else if(prior_obs_cand_idx != -1){
            prior_obs_id = obstacles[prior_obs_cand_idx].id;
        }
        if(agent_dist_to_goal < min_dist_to_goal and agent_dist_to_goal > param.goal_threshold){
            prior_obs_id = -1;
        }

//        // Cluster logger
//        if(not cluster.empty()){
//            std::string cluster_list = "";
//            for(const auto& idx : cluster){
//                cluster_list = cluster_list + std::to_string(obstacles[idx].id) + ",";
//            }
//            int highest;
//            if(highest_priority_obs_idx == -1){
//                highest = agent.id;
//            }
//            else{
//                highest = obstacles[highest_priority_obs_idx].id;
//            }
//
//            ROS_INFO_STREAM("Agent" << agent.id << ":" << cluster_list << " highest:" << highest);
//        }


        // If cluster exist and higher priority agent is in there, then yield the path
        if(prior_obs_id != -1){
            int prior_obs_idx = findObstacleIdxByObsId(prior_obs_id);
            double dist_keep = (obstacles[prior_obs_idx].radius + agent.radius) * 2.0;
            point_t z_axis(0, 0, 1);
            point_t right_hand_direction = (obstacles[prior_obs_idx].position - agent.current_state.position).cross(z_axis).normalized();

            // Check right hand rule can be used.
            double right_hand_margin;
            if(param.world_use_octomap){
                SFC sfc;
                if(M > 1){
                    sfc = constraints.getSFC(1); //TODO: SFC is not updated yet, find better approach
                }
                else{
                    sfc = constraints.getSFC(0);
                }
                right_hand_margin = sfc.raycastFromInnerPoint(agent.current_state.position, right_hand_direction);
            }
            else{
                right_hand_margin = SP_INFINITY;
            }

            for(int oi = 0; oi < obstacles.size(); oi++){
                ClosestPoints closest_points = closestPointsBetweenPointAndRay(obstacles[oi].position,
                                                                               agent.current_state.position,
                                                                               right_hand_direction);
                double margin = closest_points.closest_point2.distance(agent.current_state.position);
                if(margin < right_hand_margin){
                    right_hand_margin = margin;
                }
            }

            if(right_hand_margin > 0.1){
                agent.current_goal_position = agent.current_state.position + right_hand_direction.normalized() * std::min(right_hand_margin, dist_keep);
            }
            else{
                point_t obs_curr_position = obstacles[prior_obs_idx].position; //TODO: 여기가 문제인가? closest higher priority?
                agent.current_goal_position = agent.current_state.position +
                        (agent.current_state.position - obs_curr_position).normalized() * dist_keep;
            }
        }
        else{
            agent.current_goal_position = los_free_goal;
        }
    }

    void TrajPlanner::goalPlanningWithPriority3() {
        // Find higher_priority_agents
        std::set<int> higher_priority_agents; // set of obstacle id
        int closest_obs_idx = -1;
        double agent_dist_to_goal = agent.current_state.position.distance(agent.desired_goal_position);
        double min_dist_to_obs = SP_INFINITY;
        for(int oi = 0; oi < obstacles.size(); oi++) {
            if (obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE or
                obs_slack_indices.find(oi) != obs_slack_indices.end()) {
                higher_priority_agents.emplace(obstacles[oi].id);
                continue;
            } else if (obstacles[oi].type == ObstacleType::AGENT) {
                point_t obs_goal_position = obstacles[oi].goal_position;
                point_t obs_curr_position = obstacles[oi].position;
                point_t obs_future_position = obs_pred_trajs[oi][M - 1][n];

                // Impose the higher priority to the agents with smaller dist_to_goal
                // Do not consider the agents that have the same direction.
                // Do not consider the agent that is near goal.
                double obs_dist_to_goal = obs_curr_position.distance(obs_goal_position);
                bool obs_diff_direction = (obs_future_position - obs_curr_position).dot(obs_curr_position - agent.current_state.position) < 0;
                bool obs_stopped = obstacles[oi].velocity.norm() < param.deadlock_velocity_threshold;
                if (obs_dist_to_goal > param.goal_threshold and
                    (obs_diff_direction or obs_stopped) and
                    (agent_dist_to_goal < param.goal_threshold or obs_dist_to_goal < agent_dist_to_goal)) {
                    higher_priority_agents.emplace(obstacles[oi].id);

                    double dist_to_obs = obs_curr_position.distance(agent.current_state.position);
                    if(dist_to_obs < min_dist_to_obs){
                        min_dist_to_obs = dist_to_obs;
                        closest_obs_idx = oi;
                    }
                }
            }
        }

        if(isDeadlock()){
//        if(min_dist_to_obs < param.priority_dist_threshold){
//            double dist_keep = (obstacles[closest_obs_idx].radius + agent.radius) * 2.0;
            double dist_keep = param.priority_dist_threshold + 0.1;
            vector_t obs_to_agent_direction = agent.current_state.position - obstacles[closest_obs_idx].position;
            vector_t back_direction = obs_to_agent_direction.normalized();
            vector_t surface_direction;

            agent.current_goal_position = findProperGoalByDirection(agent.current_state.position, back_direction,
                                                                    dist_keep);
            return;
        }

        // A* considering priority
        GridBasedPlanner grid_based_planner(distmap_ptr, mission, param);
        grid_path = grid_based_planner.plan(agent.current_state.position, agent.desired_goal_position,
                                            agent.id, agent.radius, agent.downwash,
                                            obstacles, higher_priority_agents);

        if(grid_path.empty()) {
            // A* without priority
            grid_path = grid_based_planner.plan(agent.current_state.position, agent.desired_goal_position,
                                                agent.id, agent.radius, agent.downwash,
                                                obstacles);
        }

        // Find los-free goal from end of the initial trajectory
        point_t los_free_goal = grid_based_planner.findLOSFreeGoal(initial_traj[M-1][n],
                                                                   agent.desired_goal_position,
                                                                   agent.radius);

        for(int oi = 0; oi < obstacles.size(); oi++){
            point_t obs_future_position = obs_pred_trajs[oi][M-1][n];
            point_t agent_future_position = initial_traj[M-1][n];
            double margin = 0.01;
            double safety_distance = agent.radius + obstacles[oi].radius;
            if(obs_future_position.distance(agent_future_position) < safety_distance + margin){
                vector_t z_axis(0, 0, 1);
                vector_t right_hand_direction = (los_free_goal - agent.current_state.position).cross(z_axis).normalized();
                double dist_keep = std::max(safety_distance * 3.0, (los_free_goal - agent.current_state.position).norm());
//                agent.current_goal_position = findProperGoalByDirection(agent.current_state.position, right_hand_direction, dist_keep);
                agent.current_goal_position = agent.current_state.position + right_hand_direction * dist_keep;

//                agent.current_goal_position = agent.current_state.position +
//                                              (agent.desired_goal_position - agent.current_state.position).cross(z_axis);

                return;
            }
        }


        agent.current_goal_position = los_free_goal;
    }

    void TrajPlanner::obstaclePrediction(){
        size_t N_obs = obstacles.size();
        obs_pred_trajs.resize(N_obs);
        obs_pred_sizes.resize(N_obs);

        switch(param.prediction_mode){
            case PredictionMode::POSITION:
                obstaclePredictionWithCurrPos();
                break;
            case PredictionMode::LINEARKALMANFILTER:
                obstaclePredictionWithLinearKalmanFilter();
                break;
            case PredictionMode::VELOCITY:
                obstaclePredictionWithCurrVel();
                break;
            case PredictionMode::ORACLE:
                obstaclePredictionWithOracle();
                break;
            case PredictionMode::ORCA:
                obstaclePredictionWithORCA();
                break;
            case PredictionMode::PREVIOUSSOLUTION:
                obstaclePredictionWithPrevSol();
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid obstacle prediction mode");
        }

        obstaclePredictionCheck();
        obstacleSizePredictionWithConstAcc();
    }

    // Obstacle prediction using linear Kalman filter
    // It assumes that position (may not correct) is given
    //TODO: id based linear kalman filter needed
    void TrajPlanner::obstaclePredictionWithLinearKalmanFilter() {
//        if(linear_kalman_filters.size() < N_obs){
//            linear_kalman_filters.resize(N_obs);
//            for(int oi = 0; oi < N_obs; oi++){
//                linear_kalman_filters[oi].initialize(param.filter_sigma_y_sq,
//                                                     param.filter_sigma_v_sq,
//                                                     param.filter_sigma_a_sq);
//            }
//        }
//
//        obs_pred_trajs.resize(N_obs);
//        for (int oi = 0; oi < N_obs; oi++) {
//            obstacles[oi] = linear_kalman_filters[oi].filter(obstacles[oi]);
//            obs_pred_trajs[oi].resize(M);
//            for (int m = 0; m < M; m++) {
//                obs_pred_trajs[oi][m].resize(n + 1);
//                for(int i = 0; i < n + 1; i++){
//                    double m_intp = m + (double)i/n;
//                    obs_pred_trajs[oi][m][i] = obstacles[oi].position + obstacles[oi].velocity * m_intp * param.dt;
//                }
//            }
//        }
    }

    // Obstacle prediction with constant velocity assumption
    // It assumes that correct position and velocity are given
    void TrajPlanner::obstaclePredictionWithCurrVel() {
        size_t N_obs = obstacles.size();
        for (int oi = 0; oi < N_obs; oi++) {
            obs_pred_trajs[oi].resize(M);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                for(int i = 0; i < n + 1; i++){
                    double m_intp = m + (double)i/n;
                    obs_pred_trajs[oi][m][i] = obstacles[oi].position + obstacles[oi].velocity * m_intp * param.dt;
                }
            }
        }
    }

    // Obstacle prediction using perfect prediction
    void TrajPlanner::obstaclePredictionWithOracle(){
        for (int oi = 0; oi < obstacles.size(); oi++) {
            double t = (sim_current_time - obstacles[oi].start_time).toSec();
            if(obstacles[oi].type != ObstacleType::AGENT and mission.obstacles[oi]->getType() == "chasing"){
                throw std::invalid_argument("[TrajPlanner] oracle does not support chasing type obstacles");
            }

            obs_pred_trajs[oi].resize(M);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                points_t target_points;
                target_points.resize(n+1);
                std::vector<double> ts_normalized;
                ts_normalized.resize(n+1);

                if(obstacles[oi].type != ObstacleType::AGENT) {
                    for (int i = 0; i < n + 1; i++) {
                        double m_intp = m + (double) i / n;
                        dynamic_msgs::Obstacle obstacle = mission.obstacles[oi]->getObstacle(t + m_intp * param.dt);
                        target_points[i] = pointMsgToPoint3d(obstacle.pose.position);
                        ts_normalized[i] = (double) i / n;
                    }
                    points_t control_points = bernsteinFitting(target_points, ts_normalized);
                    obs_pred_trajs[oi][m] = control_points;
                }
                else {
                    // if obstacle is agent use constant velocity assumption
                    for (int i = 0; i < n + 1; i++) {
                        double m_intp = m + (double) i / n;
                        obs_pred_trajs[oi][m][i] = obstacles[oi].position + obstacles[oi].velocity * m_intp * param.dt;
                    }
                }
            }
        }
    }

    void TrajPlanner::obstaclePredictionWithCurrPos(){
        for (int oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_trajs[oi].resize(M);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                for(int i = 0; i < n + 1; i++){
                    obs_pred_trajs[oi][m][i] = obstacles[oi].position;
                }
            }
        }
    }

    void TrajPlanner::obstaclePredictionWithORCA(){
        point_t new_velocity;
        updateORCAVelocity(true);

        for (int oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_trajs[oi].resize(M);
            point_t obstacle_position = obstacles[oi].position;
            point_t orca_velocity = getObsORCAVelocity(oi);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                for(int i = 0; i < n + 1; i++){
                    double m_intp = m + (double)i/n;
                    obs_pred_trajs[oi][m][i] = obstacle_position + orca_velocity * m_intp * param.dt;
                }
            }
        }
    }

    // Dynamic obstacle -> constant velocity, Agent -> prev sol
    void TrajPlanner::obstaclePredictionWithPrevSol(){
        if(planner_seq < 2){
            obstaclePredictionWithCurrVel();
            return;
        }

        for (int oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_trajs[oi].resize(M);
            if (obstacles[oi].type != ObstacleType::AGENT) {
                // if obstacle is not agent, use current velocity to predict trajectory
                for (int m = 0; m < M; m++) {
                    for (int i = 0; i < n + 1; i++) {
                        double m_intp = m + (double) i / n;
                        obs_pred_trajs[oi][m][i] = obstacles[oi].position + obstacles[oi].velocity * m_intp * param.dt;
                    }
                }
            } else if(param.multisim_time_step == param.dt) {
                for (int m = 0; m < M; m++) {
                    obs_pred_trajs[oi][m].resize(n + 1);
                    if (m == M - 1) {
                        for(int i = 0; i < n + 1; i++){
                            obs_pred_trajs[oi][M-1][i] = obs_prev_trajs[oi][M-1][n];
                        }
                    } else {
                        obs_pred_trajs[oi][m] = obs_prev_trajs[oi][m + 1];
                    }
                }
            }
            else{
                throw std::invalid_argument("[TrajPlanner] multisim_time_step must be equal to segment time");
            }
        }
    }

    void TrajPlanner::obstaclePredictionCheck(){
        for(int oi = 0; oi < obstacles.size(); oi++) {
            point_t obs_position = obstacles[oi].position;
            if ((obs_pred_trajs[oi][0][0] - obs_position).norm() > param.multisim_reset_threshold) {
                obs_slack_indices.emplace(oi);
                for(int m = 0; m < M; m++){
                    for(int i = 0; i < n + 1; i++){
                        obs_pred_trajs[oi][m][i] = obs_position;
                    }
                }
            }
        }
    }

    void TrajPlanner::obstacleSizePredictionWithConstAcc(){
        int M_uncertainty = static_cast<int>((param.obs_uncertainty_horizon + SP_EPSILON) / param.dt);
        for(int oi = 0; oi < obstacles.size(); oi++){
            obs_pred_sizes[oi].resize(M);

            if(not param.obs_size_prediction) {
                for (int m = 0; m < M; m++) {
                    obs_pred_sizes[oi][m].resize(n + 1);
                    for(int i = 0; i < n + 1; i++){
                        obs_pred_sizes[oi][m][i] = obstacles[oi].radius;
                    }
                }
            }
            else {
                double max_acc;
                max_acc = obstacles[oi].max_acc;
                for (int m = 0; m < M_uncertainty; m++) {
                    obs_pred_sizes[oi][m].resize(n + 1);
                    Eigen::MatrixXd coef = Eigen::MatrixXd::Zero(1, n + 1);
                    coef(0, 0) = 0.5 * max_acc * pow(m * param.dt, 2);
                    coef(0, 1) = max_acc * m * pow(param.dt, 2);
                    coef(0, 2) = 0.5 * max_acc * pow(param.dt, 2);

                    Eigen::MatrixXd control_points = coef * B_inv;

                    for (int i = 0; i < n + 1; i++) {
                        obs_pred_sizes[oi][m][i] = obstacles[oi].radius + control_points(0, i);
                    }
                }
                for (int m = M_uncertainty; m < M; m++) {
                    obs_pred_sizes[oi][m].resize(n + 1);
                    for (int i = 0; i < n + 1; i++) {
                        obs_pred_sizes[oi][m][i] =
                                obstacles[oi].radius + 0.5 * max_acc * pow(M_uncertainty * param.dt, 2);
                    }
                }
            }
        }
    }

    void TrajPlanner::initialTrajPlanning() {
        switch(param.initial_traj_mode){
            case InitialTrajMode::GREEDY:
                initialTrajPlanningGreedy();
                break;
            case InitialTrajMode::ORCA:
                initialTrajPlanningORCA();
                break;
            case InitialTrajMode::POSITION:
                initialTrajPlanningCurrPos();
                break;
            case InitialTrajMode::VELOCITY:
                initialTrajPlanningCurrVel();
                break;
            case InitialTrajMode::PREVIOUSSOLUTION:
                initialTrajPlanningPrevSol();
                break;
            case InitialTrajMode::SKIP:
                // skip
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid initial traj_curr planner mode");
                break;
        }

        initialTrajPlanningCheck();
    }

    void TrajPlanner::initialTrajPlanningGreedy() {
//            //optimal
//            point_t agent.current_state.velocity(current_state.velocity.linear.x,
//                                              current_state.velocity.linear.y,
//                                              current_state.velocity.linear.z);
//            point_t current_acceleration(current_state.acceleration.linear.x,
//                                                  current_state.acceleration.linear.y,
//                                                  current_state.acceleration.linear.z);
//
//            double ideal_flight_time = (desired_goal_position - agent.current_state.position).norm() / nominal_velocity;
//            points_t initTraj_control_points;
//            initTraj_control_points.resize(n + 1);
//            initTraj_control_points[0] = agent.current_state.position;
//            initTraj_control_points[1] = agent.current_state.velocity * (ideal_flight_time / n) + initTraj_control_points[0];
//            initTraj_control_points[2] = current_acceleration * (ideal_flight_time * ideal_flight_time / (n * (n - 1)))
//                                         + initTraj_control_points[1] * 2 - initTraj_control_points[0];
//            initTraj_control_points[n - 2] = desired_goal_position;
//            initTraj_control_points[n - 1] = desired_goal_position;
//            initTraj_control_points[n] = desired_goal_position;
//
//            for (int m = 0; m < M; m++) {
//                points_t target_points;
//                target_points.resize(n+1);
//                std::vector<double> ts_normalized;
//                ts_normalized.resize(n+1);
//                for (int i = 0; i < n + 1; i++) {
//                    double m_intp = m + (double) i / n;
//                    double t_normalized = std::min(m_intp * param.dt / ideal_flight_time, 1.0);
//                    target_points[i] = getPointFromControlPoints(initTraj_control_points, t_normalized);
//                    ts_normalized[i] = (double) i / n;
//                }
//                initial_traj[m] = bernsteinFitting(target_points, ts_normalized); //TODO: infeasible near the goal point!
//            }

        //straight to goal
        double ideal_flight_time =
                (agent.current_goal_position - agent.current_state.position).norm() / agent.nominal_velocity;
        for (int m = 0; m < M; m++) {
            for (int i = 0; i < n + 1; i++) {
                double m_intp = m + (double) i / n;
                double t = std::min(m_intp * param.dt, ideal_flight_time);
                initial_traj[m][i] = agent.current_state.position +
                                        (agent.current_goal_position - agent.current_state.position).normalized() *
                                        agent.nominal_velocity * t;
            }
        }
    }

    void TrajPlanner::initialTrajPlanningPrevSol(){
        if(planner_seq < 2){
            initialTrajPlanningCurrVel();
        }
        else if(param.multisim_time_step == param.dt) {
            for (int m = 0; m < M; m++) {
                initial_traj[m].resize(n + 1);
                if (m == M - 1) {
                    for(int i = 0; i < n + 1; i++){
                        initial_traj[M - 1][i] = desired_traj[M - 1][n];
                    }
                } else {
                    initial_traj[m] = desired_traj[m + 1];
                }
            }
        }
        else{
            throw std::invalid_argument("[TrajPlanner] multisim_time_step must be equal to segment time");
        }
    }

    void TrajPlanner::initialTrajPlanningORCA(){
        updateORCAVelocity(false);

        point_t new_velocity = getAgentORCAVelocity();
        for (int m = 0; m < M; m++) {
            for (int i = 0; i < n + 1; i++) {
                double m_intp = m + (double) i / n;
                initial_traj[m][i] = agent.current_state.position + new_velocity * m_intp * param.dt;
            }
        }
    }

    void TrajPlanner::initialTrajPlanningCurrVel(){
        for (int m = 0; m < M; m++) {
            for (int i = 0; i < n + 1; i++) {
                double m_intp = m + (double) i / n;
                initial_traj[m][i] = agent.current_state.position + agent.current_state.velocity * m_intp * param.dt;
            }
        }
    }

    void TrajPlanner::initialTrajPlanningCurrPos(){
        for (int m = 0; m < M; m++) {
            for (int i = 0; i < n + 1; i++) {
                initial_traj[m][i] = agent.current_state.position;
            }
        }
    }

    void TrajPlanner::initialTrajPlanningCheck(){
        if((initial_traj[0][0] - agent.current_state.position).norm() > param.multisim_reset_threshold){
            for(int oi = 0; oi < obstacles.size(); oi++){
                obs_slack_indices.emplace(oi);
            }

            for(int m = 0; m < M; m++){
                for(int i = 0; i < n + 1; i++){
                    initial_traj[m][i] = agent.current_state.position;
                }
            }

            flag_initialize_sfc = true;
        }
    }

    void TrajPlanner::updateORCAVelocity(bool isObsPredWithORCA){
        //check orca velocity is updated
        if(isObsPredWithORCA and !orca_velocities.empty()){
            ROS_WARN("[TrajPlanner] orca velocity is already updated!, delete previous one");
            orca_velocities.clear();
        }
        else if(not orca_velocities.empty()){
            return;
        }

        if(param.world_dimension == 2){
            updateORCAVelocity2D(isObsPredWithORCA);
        }
        else{
            updateORCAVelocity3D(isObsPredWithORCA);
        }
    }

    void TrajPlanner::updateORCAVelocity2D(bool isObsPredWithORCA){
        // set agent
        std::vector<size_t> agentsList;
        agentsList.emplace_back(0);
        rvo_simulator_2d->addAgent(RVO2D::Vector2(agent.current_state.position.x(),
                                                  agent.current_state.position.y()));
        rvo_simulator_2d->setAgentRadius(0, agent.radius * param.orca_inflation_ratio);
        rvo_simulator_2d->setAgentIsDynamicObstacle(0, false);
        rvo_simulator_2d->setAgentVelocity(0, RVO2D::Vector2(agent.current_state.velocity.x(), agent.current_state.velocity.y()));

        RVO2D::Vector2 goalVector = RVO2D::Vector2(agent.current_goal_position.x(),
                                                   agent.current_goal_position.y())
                                    - rvo_simulator_2d->getAgentPosition(0);
        double agent_pref_vel = agent.max_vel[0] * param.ocra_pref_velocity_ratio;
        if (RVO2D::absSq(goalVector) > agent_pref_vel) {
            goalVector = RVO2D::normalize(goalVector) * agent_pref_vel;
        }
        rvo_simulator_2d->setAgentPrefVelocity(0, goalVector);

        // set dynamic obstacles
        size_t N_obs = obstacles.size();
        for(int oi = 0; oi < N_obs; oi++){
            rvo_simulator_2d->addAgent(RVO2D::Vector2(obstacles[oi].position.x(), obstacles[oi].position.y()));
            rvo_simulator_2d->setAgentRadius(oi + 1, obstacles[oi].radius * param.orca_inflation_ratio);
            rvo_simulator_2d->setAgentIsDynamicObstacle(oi + 1, obstacles[oi].type != ObstacleType::AGENT);
            if(isObsPredWithORCA and obstacles[oi].type == ObstacleType::AGENT){
                agentsList.emplace_back(oi + 1);
            }

            RVO2D::Vector2 obstacle_velocity = RVO2D::Vector2(obstacles[oi].velocity.x(),
                                                              obstacles[oi].velocity.y());
            rvo_simulator_2d->setAgentVelocity(oi + 1, obstacle_velocity);

            if(isObsPredWithORCA and obstacles[oi].type == ObstacleType::AGENT){
                RVO2D::Vector2 obstacle_goal_point = RVO2D::Vector2(obstacles[oi].goal_position.x(),
                                                                    obstacles[oi].goal_position.y());
                goalVector = obstacle_goal_point - rvo_simulator_2d->getAgentPosition(oi + 1);
                double obs_pref_vel = mission.agents[obstacles[oi].id].max_vel[0] * param.ocra_pref_velocity_ratio;
                if (RVO2D::absSq(goalVector) > obs_pref_vel) {
                    goalVector = RVO2D::normalize(goalVector) * obs_pref_vel;
                }
                rvo_simulator_2d->setAgentPrefVelocity(oi + 1, goalVector);
            }
            else{
                rvo_simulator_2d->setAgentPrefVelocity(oi + 1, obstacle_velocity);
            }
        }

//            // add noise
//            for(int i = 0; i < 1 + N_obs; i++){
//                float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
//                float dist = std::rand() * 0.0001f / RAND_MAX;
//                rvo_simulator->setAgentPrefVelocity(i, rvo_simulator->getAgentPrefVelocity(i) + dist * RVO::Vector3(std::cos(angle), std::sin(angle)));
//            }

        // compute new velocity
        rvo_simulator_2d->computeNewVelocity(agentsList);
        orca_velocities.resize(N_obs + 1);
        for(size_t i : agentsList){
            RVO2D::Vector2 rvo_orca_velocity = rvo_simulator_2d->getAgentVelocity(i);
            point_t orca_velocity = point_t(rvo_orca_velocity.x(), rvo_orca_velocity.y(), 0);
            orca_velocities[i] = orca_velocity;
        }

        // delete all agents and obstacles
        rvo_simulator_2d->deleteAllAgents();
    }

    void TrajPlanner::updateORCAVelocity3D(bool isObsPredWithORCA) {
        // set agent
        std::vector<size_t> agentsList;
        agentsList.emplace_back(0);
        rvo_simulator_3d->addAgent(RVO3D::Vector3(agent.current_state.position.x(),
                                                  agent.current_state.position.y(),
                                                  agent.current_state.position.z()));
        rvo_simulator_3d->setAgentRadius(0, agent.radius * param.orca_inflation_ratio);
        rvo_simulator_3d->setAgentIsDynamicObstacle(0, false);
        rvo_simulator_3d->setAgentVelocity(0, RVO3D::Vector3(agent.current_state.velocity.x(),
                                                             agent.current_state.velocity.y(),
                                                             agent.current_state.velocity.z()));
        RVO3D::Vector3 goalVector = RVO3D::Vector3(agent.current_goal_position.x(),
                                                   agent.current_goal_position.y(),
                                                   agent.current_goal_position.z())
                                    - rvo_simulator_3d->getAgentPosition(0);
        double agent_pref_vel = agent.max_vel[0] * param.ocra_pref_velocity_ratio;
        if (RVO3D::absSq(goalVector) > agent_pref_vel) {
            goalVector = RVO3D::normalize(goalVector) * agent_pref_vel;
        }
        rvo_simulator_3d->setAgentPrefVelocity(0, goalVector);


        // set dynamic obstacles
        size_t N_obs = obstacles.size();
        for (int oi = 0; oi < N_obs; oi++) {
            rvo_simulator_3d->addAgent(RVO3D::Vector3(obstacles[oi].position.x(),
                                                      obstacles[oi].position.y(),
                                                      obstacles[oi].position.z()));
            rvo_simulator_3d->setAgentRadius(oi + 1, obstacles[oi].radius * param.orca_inflation_ratio);

            rvo_simulator_3d->setAgentIsDynamicObstacle(oi + 1, obstacles[oi].type != ObstacleType::AGENT);
            if (isObsPredWithORCA and obstacles[oi].type == ObstacleType::AGENT) {
                agentsList.emplace_back(oi + 1);
            }

            RVO3D::Vector3 obstacle_velocity = RVO3D::Vector3(obstacles[oi].velocity.x(),
                                                              obstacles[oi].velocity.y(),
                                                              obstacles[oi].velocity.z());
            rvo_simulator_3d->setAgentVelocity(oi + 1, obstacle_velocity);

            if (isObsPredWithORCA) {
                RVO3D::Vector3 obstacle_goal_point = RVO3D::Vector3(obstacles[oi].goal_position.x(),
                                                                    obstacles[oi].goal_position.y(),
                                                                    obstacles[oi].goal_position.z());
                goalVector = obstacle_goal_point - rvo_simulator_3d->getAgentPosition(oi + 1);
                double obs_pref_vel = mission.agents[obstacles[oi].id].max_vel[0] * param.ocra_pref_velocity_ratio;
                if (RVO3D::absSq(goalVector) > obs_pref_vel) {
                    goalVector = RVO3D::normalize(goalVector) * obs_pref_vel;
                }
                rvo_simulator_3d->setAgentPrefVelocity(oi + 1, goalVector);
            } else {
                rvo_simulator_3d->setAgentPrefVelocity(oi + 1, obstacle_velocity);
            }
        }

//            // add noise
//            for(int i = 0; i < 1 + N_obs; i++){
//                float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
//                float dist = std::rand() * 0.0001f / RAND_MAX;
//                rvo_simulator->setAgentPrefVelocity(i, rvo_simulator->getAgentPrefVelocity(i) + dist * RVO::Vector3(std::cos(angle), std::sin(angle)));
//            }

        // compute new velocity
        rvo_simulator_3d->computeNewVelocity(agentsList);
        orca_velocities.resize(N_obs + 1);
        for (auto qi : agentsList) {
            RVO3D::Vector3 orca_velocity = rvo_simulator_3d->getAgentVelocity(qi);
            orca_velocities[qi] = point_t(orca_velocity.x(),
                                                   orca_velocity.y(),
                                                   orca_velocity.z());
        }

        // delete all agents and obstacles
        rvo_simulator_3d->deleteAllAgents();
    }

    void TrajPlanner::generateCollisionConstraints(){

        // LSC (or BVC) construction
        ros::Time lsc_start_time = ros::Time::now();
        constraints.initializeLSC(obstacles.size());
        if(param.planner_mode == PlannerMode::LSC){
            generateLSC();
        }
        else if(param.planner_mode == PlannerMode::RECIPROCALRSFC){ // Used in RAL 2021
            generateReciprocalRSFC();
        }
        else if(param.planner_mode == PlannerMode::BVC){
            generateBVC();
        }
        else{
            throw std::invalid_argument("[TrajPlanner] Invalid planner mode");
        }
        ros::Time lsc_end_time = ros::Time::now();
        planning_time.lsc_generation_time.update((lsc_end_time - lsc_start_time).toSec());

        // SFC construction
        if(param.world_use_octomap){
            ros::Time sfc_start_time = ros::Time::now();
            generateSFC();
            ros::Time sfc_end_time = ros::Time::now();
            planning_time.sfc_generation_time.update((sfc_end_time - sfc_start_time).toSec());
        }
    }

    void TrajPlanner::generateReciprocalRSFC() {
        double closest_dist;
        point_t normal_vector;

        for (int oi = 0; oi < obstacles.size(); oi++) {
            for (int m = 0; m < M; m++) {
                normal_vector = normalVector(obs_pred_trajs[oi][m][0],
                                             obs_pred_trajs[oi][m][n],
                                             initial_traj[m][0],
                                             initial_traj[m][n],
                                             closest_dist);

                std::vector<double> d;
                d.resize(n + 1);
                point_t p_init_rel_start, p_init_rel_end;
                p_init_rel_start = initial_traj[m][0] - obs_pred_trajs[oi][m][0];
                p_init_rel_end = initial_traj[m][n] - obs_pred_trajs[oi][m][n];

                for (int i = 0; i < n + 1; i++) {
                    if (obstacles[oi].type == ObstacleType::AGENT and
                        closest_dist < obs_pred_sizes[oi][m][i] + agent.radius) {
                        d[i] = 0.5 * (obs_pred_sizes[oi][m][i] + agent.radius + closest_dist);
                    } else {
                        d[i] = obs_pred_sizes[oi][m][i] + agent.radius;
                    }
                }

                // downwash
                double downwash;
                if (obstacles[oi].type == ObstacleType::AGENT) {
                    downwash = (agent.downwash * agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                               (agent.radius + obstacles[oi].radius);
                } else {
                    downwash = (agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                               (agent.radius + obstacles[oi].radius);
                }
                normal_vector.z() = normal_vector.z() / (downwash * downwash);

                constraints.setLSC(oi, m, obs_pred_trajs[oi][m], normal_vector, d);
            }
        }
    }

    void TrajPlanner::generateLSC() {
        // Linear prediction (Not used in this work)
        bool linear_prediction = param.prediction_mode == PredictionMode::VELOCITY or
                                 param.prediction_mode == PredictionMode::ORCA;
        if (linear_prediction) {
            // check whether initial traj is collision-free
            // if collision occurred, then slow down all initial traj.
            double min_collision_time = computeMinCollisionTime();

            // if collision occurred at m = 0, slow down all initial, predicted trajectories
            if (min_collision_time < M * param.dt) {
                double alpha = std::max((min_collision_time - SP_EPSILON_FLOAT) / (M * param.dt), 0.0);
                ROS_WARN_STREAM("[TrajPlanner] slow down all initial trajectories, alpha: " << alpha);
                for (int m = 0; m < M; m++) {
                    for (int i = 0; i < n + 1; i++) {
                        initial_traj[m][i] = initial_traj[0][0] + (initial_traj[m][i] - initial_traj[0][0]) * alpha;
                        for (int oi = 0; oi < obstacles.size(); oi++) {
                            obs_pred_trajs[oi][m][i] = obs_pred_trajs[oi][0][0] +
                                                       (obs_pred_trajs[oi][m][i] - obs_pred_trajs[oi][0][0]) * alpha;
                        }
                    }
                }
            }
        }

        point_t normal_vector;
        for (int oi = 0; oi < obstacles.size(); oi++) {
            // Compute downwash between agents
            double downwash;
            if (obstacles[oi].type == ObstacleType::AGENT) {
                downwash = (agent.downwash * agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                           (agent.radius + obstacles[oi].radius);
            } else {
                downwash = (agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                           (agent.radius + obstacles[oi].radius);
            }

            // Coordinate transformation
            traj_t initial_traj_trans = coordinateTransform(initial_traj, downwash);
            traj_t obs_pred_traj_trans = coordinateTransform(obs_pred_trajs[oi], downwash);

            // Normal vector planning
            for (int m = 0; m < M; m++) {
                if(linear_prediction){
                    {
                        double closest_dist;
                        normal_vector = normalVector(obs_pred_traj_trans[m][0],
                                                     obs_pred_traj_trans[m][n],
                                                     initial_traj_trans[m][0],
                                                     initial_traj_trans[m][n],
                                                     closest_dist);
                    }
                } else {
                    // Compute normal vector of LSC
                    normal_vector = normalVectorBetweenPolys(initial_traj_trans[m], obs_pred_traj_trans[m]);
                }

                // Compute safety margin
                std::vector<double> d;
                d.resize(n + 1);
                if (obstacles[oi].type == ObstacleType::AGENT or obs_slack_indices.find(oi) == obs_slack_indices.end()) {
                    for (int i = 0; i < n + 1; i++) {
                        double epsilon_n_max = 0.0;
                        double collision_dist = obstacles[oi].radius + agent.radius;
                        d[i] = 0.5 * (collision_dist +
                                      (initial_traj_trans[m][i] - obs_pred_traj_trans[m][i]).dot(normal_vector));
                    }
                } else {
                    // If the disturbance is too large, use reciprocal RSFC
                    for (int i = 0; i < n + 1; i++) {
                        d[i] = obs_pred_sizes[oi][m][i] + agent.radius;
                    }
                }

                // Return to original coordinatation
                normal_vector.z() = normal_vector.z() / downwash;
                constraints.setLSC(oi, m, obs_pred_trajs[oi][m], normal_vector, d);
            }
        }
    }

    void TrajPlanner::generateBVC(){
        // Since the original BVC does not consider downwash, we conduct coordinate transformation to consider downwash.
        point_t normal_vector;
        for (int oi = 0; oi < obstacles.size(); oi++) {
            // Compute downwash
            double downwash = (agent.downwash * agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                    (agent.radius + obstacles[oi].radius);

            // Coordinate transformation
            traj_t initial_traj_trans = coordinateTransform(initial_traj, downwash);
            traj_t obs_pred_traj_trans = coordinateTransform(obs_pred_trajs[oi], downwash);

            // normal vector
            normal_vector = (initial_traj_trans[0][0] - obs_pred_traj_trans[0][0]).normalized();

            // safety margin
            std::vector<double> d;
            d.resize(n + 1);
            for (int i = 0; i < n + 1; i++) {
                double collision_dist = obstacles[oi].radius + agent.radius;
                d[i] = 0.5 * (collision_dist +
                              (initial_traj_trans[0][0] - obs_pred_traj_trans[0][0]).dot(normal_vector));
            }

            // Return to original coordinatation
            normal_vector.z() = normal_vector.z() / downwash;

            for (int m = 0; m < M; m++) {
                constraints.setLSC(oi, m, obs_pred_trajs[oi][m], normal_vector, d);
            }
        }
    }

    void TrajPlanner::generateSFC(){
        if(flag_initialize_sfc){
            constraints.initializeSFC(agent.current_state.position, agent.radius);
            flag_initialize_sfc = false;
        }
        else{
            constraints.generateFeasibleSFC(initial_traj[M-1][n], agent.current_goal_position,
                                            grid_path, agent.radius);
        }
    }

    bool TrajPlanner::trajOptimization() {
        Timer timer;

        // Solve QP problem using CPLEX
        timer.reset();
        try{
            desired_traj = traj_optimizer->solve(agent, constraints, traj_cost);
        }
        catch(...){
            // debug code
            for(int m = 0; m < M; m++){
                SFC sfc = constraints.getSFC(m);
                for(const auto& control_point : initial_traj[m]){
                    bool check1 = sfc.isPointInSFC(control_point);
                    if(not check1){
                        ROS_ERROR("[TrajPlanner] SFC constraint is not feasible.");
                    }
                }
                for(int oi = 0; oi < obstacles.size(); oi++){
                    for(int i = 0; i < n + 1; i++){
                        LSC lsc = constraints.getLSC(oi, m, i);
                        double margin = (initial_traj[m][i] - lsc.obs_control_point).dot(lsc.normal_vector) - lsc.d;
                        bool check2 = margin > 0;
                        if(not check2){
                            ROS_ERROR("[TrajPlanner] LSC constraint is not feasible.");
                        }
                    }
                }
            }
        }
        timer.stop();
        planning_time.traj_optimization_time.update(timer.elapsedSeconds());

        return true;
    }

    void TrajPlanner::publishCollisionConstraints() {
        dynamic_msgs::CollisionConstraint msg_collision_constraints_raw;
        visualization_msgs::MarkerArray msg_collision_constraints_vis;
        msg_collision_constraints_raw = constraints.convertToRawMsg(obstacles, planner_seq);
        msg_collision_constraints_vis = constraints.convertToMarkerArrayMsg(obstacles, mission.color, agent.id, agent.radius);

        pub_collision_constraints_raw.publish(msg_collision_constraints_raw);
        pub_collision_constraints_vis.publish(msg_collision_constraints_vis);
    }

    void TrajPlanner::publishInitialTraj() {
        if(param.initial_traj_mode == InitialTrajMode::SKIP){
            return;
        }

        //Raw
        dynamic_msgs::TrajectoryArray msg_initial_traj_raw;
        msg_initial_traj_raw.planner_seq = planner_seq;
        msg_initial_traj_raw.trajectories.emplace_back(trajToTrajMsg(initial_traj, agent.id, param.dt));

        //Vis
        //TODO: it publishes only segment points
        visualization_msgs::MarkerArray msg_initial_traj_vis;
        msg_initial_traj_vis.markers.clear();
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 0.5;
        for (int m = 0; m < M; m++) {
            marker.id = m;
            marker.pose.position = point3DToPointMsg(initial_traj[m][0]);
            msg_initial_traj_vis.markers.emplace_back(marker);
        }
        {
            marker.id = M;
            marker.pose.position = point3DToPointMsg(initial_traj[M - 1][n]);
            msg_initial_traj_vis.markers.emplace_back(marker);
        }

        pub_initial_traj_raw.publish(msg_initial_traj_raw);
        pub_initial_traj_vis.publish(msg_initial_traj_vis);
    }

    void TrajPlanner::publishGridPath(){
        visualization_msgs::MarkerArray msg_grid_path_vis;
        msg_grid_path_vis.markers.clear();
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        msg_grid_path_vis.markers.emplace_back(marker);

        pub_grid_path.publish(msg_grid_path_vis);

        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color = mission.color[agent.id];
        marker.color.a = 0.2;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        int marker_id = 0;
        for(const auto& point : grid_path) {
            marker.id = marker_id++;
            marker.pose.position = point3DToPointMsg(point);
            marker.pose.orientation = defaultQuaternion();
            msg_grid_path_vis.markers.emplace_back(marker);
        }

        pub_grid_path.publish(msg_grid_path_vis);
    }

    void TrajPlanner::publishObstaclePrediction() {
        // obstacle prediction vis
        visualization_msgs::MarkerArray msg_obs_pred_traj_vis;
        msg_obs_pred_traj_vis.markers.clear();
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.color.a = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        for (int oi = 0; oi < obstacles.size(); oi++) {
            for (int m = 0; m <= M; m++) {
                marker.id = (M+1) * oi + m;
                marker.ns = std::to_string(m);

                int m_, i;
                if(m < M) {
                    m_ = m;
                    i = 0;
                }
                else{
                    m_ = M - 1;
                    i = n;
                }

//                double downwash;
//                if(obstacles[oi].type == ObstacleType::AGENT){
//                    downwash = (agent.downwash * agent.radius + obstacles[oi].downwash * obstacles[oi].radius)/(agent.radius + obstacles[oi].radius);
//                }
//                else{
//                    downwash = (agent.radius + obstacles[oi].downwash * obstacles[oi].radius)/(agent.radius + obstacles[oi].radius);
//                }
//                marker.scale.x = 2 * (obs_pred_sizes[oi][m_][i] + agent.radius);
//                marker.scale.y = 2 * (obs_pred_sizes[oi][m_][i] + agent.radius);
//                marker.scale.z = 2 * (obs_pred_sizes[oi][m_][i] + agent.radius) * downwash;

                marker.scale.x = 2 * obs_pred_sizes[oi][m_][i];
                marker.scale.y = 2 * obs_pred_sizes[oi][m_][i];
                marker.scale.z = 2 * obs_pred_sizes[oi][m_][i] * obstacles[oi].downwash;

                marker.pose.position = point3DToPointMsg(obs_pred_trajs[oi][m_][i]);
                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = 0;
                marker.pose.orientation.w = 1;
                msg_obs_pred_traj_vis.markers.emplace_back(marker);
            }
        }
        pub_obs_pred_traj_vis.publish(msg_obs_pred_traj_vis);

        // obstacle prediction raw
        size_t N_obs = obstacles.size();
        dynamic_msgs::TrajectoryArray msg_obs_pred_traj_raw;
        msg_obs_pred_traj_raw.planner_seq = planner_seq;
        msg_obs_pred_traj_raw.trajectories.resize(N_obs);
        for(int oi = 0; oi < N_obs; oi++){
            msg_obs_pred_traj_raw.trajectories[oi] = trajToTrajMsg(obs_pred_trajs[oi], obstacles[oi].id, param.dt);
        }
        pub_obs_pred_traj_raw.publish(msg_obs_pred_traj_raw);
    }

    bool TrajPlanner::isDeadlock() const {
        //If agent's velocity is lower than some threshold, then it determines agent is in deadlock
        double dist_to_goal = (agent.current_state.position - agent.desired_goal_position).norm();
        return planner_seq > param.deadlock_seq_threshold and
               agent.current_state.velocity.norm() < param.deadlock_velocity_threshold and
               dist_to_goal > param.goal_threshold;
    }

    int TrajPlanner::findObstacleIdxByObsId(int obs_id) const{
        int oi = -1;
        for(int i = 0; i < obstacles.size(); i++){
            if(obstacles[i].id == obs_id){
                oi = i;
            }
        }
        return oi;
    }

    double TrajPlanner::distanceToGoalByObsId(int obs_id) const{
        int obs_idx = findObstacleIdxByObsId(obs_id);
        return distanceToGoalByObsIdx(obs_idx);
    }

    double TrajPlanner::distanceToGoalByObsIdx(int obs_idx) const{
        return obstacles[obs_idx].goal_position.distance(obstacles[obs_idx].position);
    }

    double TrajPlanner::computeCollisionTimeToDistmap(const point_t& start_position,
                                                      const point_t& goal_position,
                                                      double agent_radius,
                                                      double time_horizon){
        double collision_time = 0;
        bool isCollided = distmap_ptr->getDistance(start_position) < agent_radius;
        if(goal_position == start_position){
            if(isCollided){
                collision_time = 0;
            }
            else{
                collision_time = SP_INFINITY;
            }
            return collision_time;
        }

        double search_time_step = 0.1; //TODO: parameterization
        double current_time = 0;
        point_t current_search_point;
        while(!isCollided and current_time < time_horizon){
            current_time += search_time_step;
            current_search_point = start_position + (goal_position - start_position) * (current_time / time_horizon);
            isCollided = distmap_ptr->getDistance(current_search_point) < agent_radius;
        }

        if(isCollided){
            collision_time = current_time;
        }
        else{
            collision_time = SP_INFINITY;
        }

        return collision_time;
    }

    double TrajPlanner::computeMinCollisionTime(){
        //TODO: only support constant velocity, implement polynomial min collision time
        double collision_time, min_collision_time = SP_INFINITY;
        double total_time_horizon = M * param.dt;

        if(param.world_use_octomap){
            collision_time = computeCollisionTimeToDistmap(initial_traj[0][0],
                                                           initial_traj[M - 1][n],
                                                           agent.radius,
                                                           total_time_horizon);
            if (min_collision_time > collision_time) {
                min_collision_time = collision_time;
            }
        }

        size_t N_obs = obstacles.size();
        for (int oi = 0; oi < N_obs; oi++) {
            collision_time = computeCollisionTime(obs_pred_trajs[oi][0][0],
                                                  obs_pred_trajs[oi][M - 1][n],
                                                  initial_traj[0][0],
                                                  initial_traj[M - 1][n],
                                                  obstacles[oi].radius + agent.radius,
                                                  total_time_horizon);

            if (min_collision_time > collision_time) {
                min_collision_time = collision_time;
            }

            if(param.world_use_octomap){
                collision_time = computeCollisionTimeToDistmap(obs_pred_trajs[oi][0][0],
                                                               obs_pred_trajs[oi][M - 1][n],
                                                               obstacles[oi].radius,
                                                               total_time_horizon);
                if (min_collision_time > collision_time) {
                    min_collision_time = collision_time;
                }
            }

            for (int oj = 0; oj < N_obs; oj++) {
                if (oj > oi) {
                    collision_time = computeCollisionTime(obs_pred_trajs[oi][0][0],
                                                          obs_pred_trajs[oi][M - 1][n],
                                                          obs_pred_trajs[oj][0][0],
                                                          obs_pred_trajs[oj][M - 1][n],
                                                          obstacles[oi].radius + obstacles[oj].radius,
                                                          total_time_horizon);
                } else {
                    continue;
                }

                if (min_collision_time > collision_time) {
                    min_collision_time = collision_time;
                }
            }
        }

        return min_collision_time;
    }

    point_t TrajPlanner::normalVector(const point_t& obs_start, const point_t& obs_goal,
                                      const point_t& agent_start, const point_t& agent_goal,
                                      double& closest_dist) {
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenLinePaths(obs_start, obs_goal, agent_start, agent_goal);
        closest_dist = closest_points.dist;

        point_t delta, normal_vector;
        delta = closest_points.closest_point2 - closest_points.closest_point1;
        normal_vector = delta.normalized();
        if (normal_vector.norm() == 0) {
            ROS_WARN("[Util] heuristic method was used to get normal vector");
            point_t a, b;
            a = agent_start - obs_start;
            b = agent_goal - obs_goal;
            if(a.norm() == 0 and b.norm() == 0){
                normal_vector = point_t(1, 0, 0);
            }
            else{
                normal_vector = (b - a).cross(point_t(0, 0, 1));
            }
        }
        return normal_vector;
    }

    point_t TrajPlanner::normalVectorBetweenPolys(const points_t &control_points_agent,
                                                  const points_t &control_points_obs) {
        size_t n_control_points = control_points_agent.size();
        points_t control_points_rel;
        control_points_rel.resize(n_control_points);
        for(size_t i = 0; i < n_control_points; i++){
            control_points_rel[i] = control_points_agent[i] - control_points_obs[i];
        }

        ClosestPoints closest_points = closestPointsBetweenPointAndConvexHull(point_t(0,0,0),
                                                                              control_points_rel);
        point_t normal_vector = closest_points.closest_point2.normalized();
        return normal_vector;
    }

    point_t TrajPlanner::findProperGoalByDirection(const point_t& start, const vector_t& direction, double dist_keep){
        point_t goal;
        vector_t surface_direction;
        double margin;

        if(param.world_use_octomap and planner_seq > 1){
            SFC sfc = constraints.findProperSFC(start, start + direction * dist_keep);
            margin = sfc.raycastFromInnerPoint(agent.current_state.position, direction, surface_direction);
        }
        else{
            margin = SP_INFINITY;
        }

        if(margin > dist_keep){
            goal = agent.current_state.position + direction * dist_keep;
        }
        else{
            point_t wall_direction = (direction - surface_direction * surface_direction.dot(direction)).normalized();
            goal = agent.current_state.position + direction * margin + wall_direction * (dist_keep - margin);
        }

        return goal;
    }
}