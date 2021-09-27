#include <traj_planner.hpp>

namespace DynamicPlanning{
    TrajPlanner::TrajPlanner(int _agent_id,
                             const ros::NodeHandle& _nh,
                             const Param& _param,
                             const Mission& _mission) : nh(_nh), param(_param), mission(_mission)
    {
        // Initialize parameters
        param.initialize(nh);

        // Initialize agent
        agent = mission.agents[_agent_id];
        agent.current_state.position = mission.agents[_agent_id].start_position;

        // Initialize callback flags
        flag_current_state.is_updated = false;
        flag_obstacles.is_updated = false;

        // Initialize trajectory param, offsets
        dim = param.world_dimension;
        M = static_cast<int>((param.horizon + SP_EPSILON) / param.dt);
        n = param.n;
        phi = param.phi;
        if(param.N_constraint_segments < 0){
            param.N_constraint_segments = M;
        }

        // Initialize initial trajectory
        initial_traj.resize(M);
        for(int m = 0; m < M; m++){
            initial_traj[m].resize(n + 1);
        }

        // Initialize control points of trajectory
        traj_curr.resize(M);
        for (int m = 0; m < M; m++) {
            traj_curr[m].resize(n + 1);
        }

        // Initialize planner states
        planner_state = PlannerState::WAIT;
        planner_seq = 0;
        planning_report = PlanningReport::Initialized;
        N_obs = 0;
        deadlock_start_seq = 0;
        sol_diff = 0;
        current_qp_cost = 0;
        flag_initialize_sfc = true;

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
    }

    void TrajPlanner::initializeROS() {
        // Initialize ros publisher and subscriber
        std::string prefix = "/mav";
        sub_current_state = nh.subscribe(prefix + std::to_string(agent.id) + "_current_state", 1,
                                         &TrajPlanner::currentStateCallback, this);
        sub_obstacles = nh.subscribe(prefix + std::to_string(agent.id) + "_obstacles", 1,
                                     &TrajPlanner::obstaclesCallback, this);
        pub_collision_constraints_raw = nh.advertise<dynamic_msgs::CollisionConstraint>(
                prefix + std::to_string(agent.id) + "_collision_constraints_raw", 1);
        pub_collision_constraints_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + std::to_string(agent.id) + "_collision_constraints_vis", 1);
        pub_initial_traj_raw = nh.advertise<dynamic_msgs::TrajectoryArray>(
                prefix + std::to_string(agent.id) + "_initial_traj_raw", 1);
        pub_initial_traj_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + std::to_string(agent.id) + "_initial_traj_vis", 1);
        pub_obs_pred_traj_raw = nh.advertise<dynamic_msgs::TrajectoryArray>(
                prefix + std::to_string(agent.id) + "_obs_pred_traj_raw", 1);
        pub_obs_pred_traj_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + std::to_string(agent.id) + "_obs_pred_traj_vis", 1);
        pub_grid_path = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + std::to_string(agent.id) + "_grid_path_vis", 1);
        obstacle_update_time = ros::Time::now();
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

        // Update clock
        sim_current_time = _sim_current_time;

        // Stop at debug_stop_seq (debugging purpose)
        if(planner_seq == param.debug_stop_seq){
            if(planner_seq == 0){
                planning_report = PlanningReport::Initialized;
            }
            else{
                planning_report = PlanningReport::WAITFORROSMSG;
            }
            return planning_report;
        }

        // Start planning
        planner_seq++;
        ros::Time planning_start_time = ros::Time::now();
        planImpl();

        // Re-initialization for replanning
        flag_obstacles.is_updated = false;
        flag_current_state.is_updated = false;
        orca_velocities.clear();
        obs_prev_trajs.clear();
        traj_prev = traj_curr;

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
        currentStateCallback(msg_current_state);
    }

    void TrajPlanner::setDistMap(const std::shared_ptr<DynamicEDTOctomap>& _distmap_obj){
        distmap_obj = _distmap_obj;
    }

    void TrajPlanner::setObstacles(const dynamic_msgs::ObstacleArray& msg_dynamic_obstacles){
        obstaclesCallback(msg_dynamic_obstacles);
    }

    void TrajPlanner::setObsPrevTrajs(const std::vector<traj_t>& _obs_prev_trajs) {
        obs_prev_trajs = _obs_prev_trajs;
    }

    void TrajPlanner::setStart(const octomap::point3d& new_start_position) {
        mission.agents[agent.id].start_position = new_start_position;
        agent.start_position = new_start_position;
    }

    void TrajPlanner::setDesiredGoal(const octomap::point3d& new_desired_goal_position) {
        mission.agents[agent.id].desired_goal_position = new_desired_goal_position;
        agent.desired_goal_position = new_desired_goal_position;
    }

    void TrajPlanner::reset(const dynamic_msgs::State& msg_current_state) {
        // Reset current state
        currentStateCallback(msg_current_state);

        // Reset current trajectory
        for(int m = 0; m < M; m++){
            for(int i = 0; i < n + 1; i++){
                traj_curr[m][i] = agent.current_state.position;
            }
        }

        // Reset SFC
        CorridorConstructor corridor_constructor(distmap_obj, mission, param);
        Box box = corridor_constructor.expandBoxFromPoint(agent.current_state.position,
                                                          agent.current_goal_position,
                                                          agent.radius);
        for(int m = 0; m < M; m++){
            constraints.setSFC(m, box);
        }
    }

    void TrajPlanner::updatePlannerState(const PlannerState& new_planner_state){
        planner_state = new_planner_state;
    }

    octomap::point3d TrajPlanner::getCurrentPosition() const{
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
        dynamic_msgs::State msg_current_state = getStateFromControlPoints(traj_curr,
                                                                          future_time, M, n, param.dt);
        msg_current_state.planner_seq = planner_seq;
        return msg_current_state;
    }

    octomap::point3d TrajPlanner::getAgentORCAVelocity() const{
        return orca_velocities[0];
    }

    octomap::point3d TrajPlanner::getObsORCAVelocity(int oi) const{
        if(obstacles.size() < oi + 2){
            throw std::invalid_argument("[TrajPlanner] orca velocity is not updated yet");
        }

        if(obstacles[oi].type == ObstacleType::AGENT){
            return orca_velocities[oi + 1]; // orca_velocities[0] = agent's orca velocity
        }
        else if(obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
            return octomap::point3d(0, 0, 0);
        }
        else if(obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE){
            octomap::point3d obstacle_velocity;
            if(param.world_dimension == 2){
                obstacle_velocity = octomap::point3d(obstacles[oi].velocity.linear.x,
                                                     obstacles[oi].velocity.linear.y,
                                                     0);
            }
            else{
                obstacle_velocity = vector3MsgToPoint3d(obstacles[oi].velocity.linear);
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

    double TrajPlanner::getQPCost() const{
        return current_qp_cost;
    }

    PlanningReport TrajPlanner::getPlanningReport() const{
        return planning_report;
    }

    traj_t TrajPlanner::getTraj() const{
        return traj_curr;
    }

    octomap::point3d TrajPlanner::getNormalVector(int obs_id, int m) const{
        int oi = findObstacleIdxById(obs_id);
        if(oi == -1){
            throw std::invalid_argument("[TrajPlanner] invalid mav_id for normalVector");
        }

        return constraints.getLSC(oi, m, 0).normal_vector;
    }

    octomap::point3d TrajPlanner::getCurrentGoalPosition() const{
        return agent.current_goal_position;
    }

    octomap::point3d TrajPlanner::getDesiredGoalPosition() const{
        return agent.desired_goal_position;
    }

    int TrajPlanner::getPlannerSeq() const{
        return planner_seq;
    }

    void TrajPlanner::currentStateCallback(const dynamic_msgs::State& msg_current_state){
        // update agent.current_state
        if(param.world_dimension == 2){
            agent.current_state.position = octomap::point3d(msg_current_state.pose.position.x,
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

    void TrajPlanner::obstaclesCallback(const dynamic_msgs::ObstacleArray& msg_obstacles) {
        obstacles.clear();
        if(not flag_obstacles.is_updated){
            obstacle_start_time = msg_obstacles.start_time;
        }
        if(obstacle_update_time < msg_obstacles.header.stamp){
            obstacle_update_time = msg_obstacles.header.stamp;
        }
        else{
            flag_obstacles.is_updated = false;
            return;
        }

        N_obs = msg_obstacles.obstacles.size();
        obstacles.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            obstacles[oi] = msg_obstacles.obstacles[oi];
        }
        flag_obstacles.is_updated = true;
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

        octomap::point3d new_velocity = getAgentORCAVelocity();
        for(int m = 0; m < M; m++){
            for(int i = 0; i < n + 1; i++){
                double m_intp = m + (double)i/n;
                traj_curr[m][i] = agent.current_state.position + new_velocity * m_intp * param.dt;
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
            // Check endpoint for deadlock detection (Not used in this work.)
            if (planner_seq > param.deadlock_seq_threshold and
                (traj_curr[M - 1][n] - traj_prev[M - 1][n]).norm() < SP_EPSILON_FLOAT and
                (traj_curr[M - 1][n] - agent.desired_goal_position).norm() > param.goal_threshold) {

                if (deadlock_start_seq == 0) {
                    deadlock_start_seq = planner_seq;
                    deadlock_endpoint = traj_curr[M - 1][n];
                }
                else if ((traj_curr[M - 1][n] - deadlock_endpoint).norm() > SP_EPSILON_FLOAT) {
                    deadlock_start_seq = planner_seq;
                    deadlock_endpoint = traj_curr[M - 1][n];
                }
            }

            // Save difference between traj_prev and traj_curr (Not used in this work)
            if(planner_seq > 2){
                sol_diff = 0;
                for(int m = 0; m < M; m++){
                    for(int i = 0; i < n + 1; i++){
                        sol_diff += (traj_curr[m][i] - traj_prev[m][i]).norm();
                    }
                }
            }
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

        if(param.world_use_octomap and distmap_obj == nullptr){
            throw std::invalid_argument("[TrajPlanner] distmap is not ready");
        }
    }

    void TrajPlanner::goalPlanning() {
        // Patrol
        if (planner_state == PlannerState::PATROL and
        (agent.desired_goal_position - agent.current_state.position).norm() < param.goal_threshold) {
            // Swap start and goal position
            octomap::point3d temp = agent.desired_goal_position;
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
        octomap::point3d orca_velocity = getAgentORCAVelocity();
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
            octomap::point3d z_axis(0, 0, 1);
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
        for(int oi = 0; oi < N_obs; oi++){
            if(obs_slack_indices.find(oi) != obs_slack_indices.end()){
                high_priority_obstacle_ids.emplace(obstacles[oi].id);
                continue;
            }

            if(obstacles[oi].type == ObstacleType::AGENT) {
                octomap::point3d obs_goal_position = pointMsgToPoint3d(obstacles[oi].goal_point);
                octomap::point3d obs_curr_position = pointMsgToPoint3d(obstacles[oi].pose.position);
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
            octomap::point3d obs_curr_position = pointMsgToPoint3d(obstacles[closest_obs_id].pose.position);
            agent.current_goal_position = agent.current_state.position -
                                          (obs_curr_position - agent.current_state.position).normalized() * dist_keep;
            return;
        }

        // A* considering priority
        GridBasedPlanner grid_based_planner(distmap_obj, mission, param);
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
        grid_los_goal = grid_based_planner.findLOSFreeGoal(initial_traj[M-1][n],
                                                           agent.desired_goal_position,
                                                           obstacles,
                                                           agent.radius);

        agent.current_goal_position = grid_los_goal;
    }

    void TrajPlanner::obstaclePrediction(){
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
                break;
        }

        obstaclePredictionCheck();
        obstacleSizePredictionWithConstAcc();
    }

    // Obstacle prediction using linear Kalman filter
    // It assumes that position (may not correct) is given
    void TrajPlanner::obstaclePredictionWithLinearKalmanFilter() {
        if(linear_kalman_filters.size() < N_obs){
            linear_kalman_filters.resize(N_obs);
            for(int oi = 0; oi < N_obs; oi++){
                linear_kalman_filters[oi].initialize(param.filter_sigma_y_sq,
                                                     param.filter_sigma_v_sq,
                                                     param.filter_sigma_a_sq);
            }
        }

        obs_pred_trajs.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            obstacles[oi] =
                    linear_kalman_filters[oi].filter(obstacles[oi], obstacle_update_time);
            obs_pred_trajs[oi].resize(M);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                for(int i = 0; i < n + 1; i++){
                    double m_intp = m + (double)i/n;
                    obs_pred_trajs[oi][m][i] = pointMsgToPoint3d(obstacles[oi].pose.position) +
                                              vector3MsgToPoint3d(obstacles[oi].velocity.linear) * m_intp * param.dt;
                }
            }
        }

//        obs_pred_sizes.resize(N_obs);
//        int M_uncertainty = static_cast<int>((param.uncertainty_horizon + SP_EPSILON) / param.dt);
//        for(int oi = 0; oi < N_obs; oi++){
//            obs_pred_sizes[oi].resize(M);
//            double max_acc = obstacles[oi].max_acc;
//            for (int m = 0; m < M_uncertainty; m++) {
//                obs_pred_sizes[oi][m].resize(n + 1);
//                Eigen::MatrixXd coef = Eigen::MatrixXd::Zero(1, n+1);
//                coef(0, 0) = 0.5 * max_acc * pow(m * param.dt, 2);
//                coef(0, 1) = max_acc * m * pow(param.dt, 2);
//                coef(0, 2) = 0.5 * max_acc * pow(param.dt, 2);
//
//                Eigen::MatrixXd control_points = coef * B_inv;
//                for(int i = 0; i < n + 1; i++){
//                    double m_intp = m + (double)i/n;
//                    double uncertainty_radius =
//                            linear_kalman_filters[oi].getUncertaintyRadius(m_intp * param.dt);
////                        obs_pred_sizes[oi][m][i] = obstacles[oi].size + control_points(0, i);
//                    obs_pred_sizes[oi][m][i] = obstacles[oi].radius + uncertainty_radius;
//                }
//            }
//            for(int m = M_uncertainty; m < M; m++){
//                obs_pred_sizes[oi][m].resize(n + 1);
//                for(int i = 0; i < n + 1; i++){
////                        obs_pred_sizes[oi][m][i] = obstacles[oi].size + 0.5 * max_acc * pow(M_uncertainty * param.dt, 2);
//                    obs_pred_sizes[oi][m][i] = obs_pred_sizes[oi][std::max(M_uncertainty - 1, 0)][n];
//                }
//            }
//        }
    }

    // Obstacle prediction with constant velocity assumption
    // It assumes that correct position and velocity are given
    void TrajPlanner::obstaclePredictionWithCurrVel() {
        obs_pred_trajs.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            obs_pred_trajs[oi].resize(M);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                for(int i = 0; i < n + 1; i++){
                    double m_intp = m + (double)i/n;
                    obs_pred_trajs[oi][m][i] = pointMsgToPoint3d(obstacles[oi].pose.position) +
                                              vector3MsgToPoint3d(obstacles[oi].velocity.linear) * m_intp * param.dt;
                }
            }
        }
    }

    // Obstacle prediction using perfect prediction
    void TrajPlanner::obstaclePredictionWithOracle(){
        double t = (sim_current_time - obstacle_start_time).toSec();
        obs_pred_trajs.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            if(obstacles[oi].type != ObstacleType::AGENT and mission.obstacles[oi]->getType() == "chasing"){
                throw std::invalid_argument("[TrajPlanner] oracle does not support chasing type obstacles");
            }

            obs_pred_trajs[oi].resize(M);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                std::vector<octomap::point3d> target_points;
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
                    std::vector<octomap::point3d> control_points = bernsteinFitting(target_points, ts_normalized);
                    obs_pred_trajs[oi][m] = control_points;
                }
                else {
                    // if obstacle is agent use constant velocity assumption
                    for (int i = 0; i < n + 1; i++) {
                        double m_intp = m + (double) i / n;
                        obs_pred_trajs[oi][m][i] = pointMsgToPoint3d(obstacles[oi].pose.position) +
                                                  vector3MsgToPoint3d(obstacles[oi].velocity.linear) * m_intp *
                                                  param.dt;
                    }
                }
            }
        }

//        obs_pred_sizes.resize(N_obs);
//        for(int oi = 0; oi < N_obs; oi++){
//            obs_pred_sizes[oi].resize(M);
//
//            if(obstacles[oi].type == ObstacleType::AGENT and param.obs_size_prediction) {
//                double max_acc = obstacles[oi].max_acc;
//                int M_uncertainty = static_cast<int>((param.uncertainty_horizon + SP_EPSILON) / param.dt);
//                for (int m = 0; m < M_uncertainty; m++) {
//                    obs_pred_sizes[oi][m].resize(n + 1);
//                    Eigen::MatrixXd coef = Eigen::MatrixXd::Zero(1, n + 1);
//                    coef(0, 0) = 0.5 * max_acc * pow(m * param.dt, 2);
//                    coef(0, 1) = max_acc * m * pow(param.dt, 2);
//                    coef(0, 2) = 0.5 * max_acc * pow(param.dt, 2);
//
//                    Eigen::MatrixXd control_points = coef * B_inv;
//
//                    for (int i = 0; i < n + 1; i++) {
//                        obs_pred_sizes[oi][m][i] = obstacles[oi].radius + control_points(0, i);
//                    }
//                }
//                for (int m = M_uncertainty; m < M; m++) {
//                    obs_pred_sizes[oi][m].resize(n + 1);
//                    for (int i = 0; i < n + 1; i++) {
//                        obs_pred_sizes[oi][m][i] =
//                                obstacles[oi].radius + 0.5 * max_acc * pow(M_uncertainty * param.dt, 2);
//                    }
//                }
//            } else{
//                for (int m = 0; m < M; m++) {
//                    obs_pred_sizes[oi][m].resize(n + 1);
//                    for(int i = 0; i < n + 1; i++){
//                        if(param.obs_size_prediction) {
//                            obs_pred_sizes[oi][m][i] = obstacles[oi].radius;
//                        }
//                        else{
//                            obs_pred_sizes[oi][m][i] = obstacles[oi].radius + 0.1; //TODO: msg delay error compensation
//                        }
//                    }
//                }
//            }
//        }
    }

    void TrajPlanner::obstaclePredictionWithCurrPos(){
        obs_pred_trajs.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            obs_pred_trajs[oi].resize(M);
            for (int m = 0; m < M; m++) {
                obs_pred_trajs[oi][m].resize(n + 1);
                for(int i = 0; i < n + 1; i++){
                    obs_pred_trajs[oi][m][i] = pointMsgToPoint3d(obstacles[oi].pose.position);
                }
            }
        }
    }

    void TrajPlanner::obstaclePredictionWithORCA(){
        octomap::point3d new_velocity;
        updateORCAVelocity(true);

        obs_pred_trajs.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            obs_pred_trajs[oi].resize(M);
            octomap::point3d obstacle_position = pointMsgToPoint3d(obstacles[oi].pose.position);
            octomap::point3d orca_velocity = getObsORCAVelocity(oi);
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

        obs_pred_trajs.resize(N_obs);
        for (int oi = 0; oi < N_obs; oi++) {
            obs_pred_trajs[oi].resize(M);
            if (obstacles[oi].type != ObstacleType::AGENT) {
                // if obstacle is not agent, use current velocity to predict trajectory
                for (int m = 0; m < M; m++) {
                    for (int i = 0; i < n + 1; i++) {
                        double m_intp = m + (double) i / n;
                        obs_pred_trajs[oi][m][i] = pointMsgToPoint3d(obstacles[oi].pose.position) +
                                                   vector3MsgToPoint3d(obstacles[oi].velocity.linear) * m_intp *
                                                   param.dt;
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
        for(int oi = 0; oi < N_obs; oi++) {
            octomap::point3d obs_position = pointMsgToPoint3d(obstacles[oi].pose.position);
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
        obs_pred_sizes.resize(N_obs);
        int M_uncertainty = static_cast<int>((param.obs_uncertainty_horizon + SP_EPSILON) / param.dt);
        for(int oi = 0; oi < N_obs; oi++){
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
//            octomap::point3d agent.current_state.velocity(current_state.velocity.linear.x,
//                                              current_state.velocity.linear.y,
//                                              current_state.velocity.linear.z);
//            octomap::point3d current_acceleration(current_state.acceleration.linear.x,
//                                                  current_state.acceleration.linear.y,
//                                                  current_state.acceleration.linear.z);
//
//            double ideal_flight_time = (desired_goal_position - agent.current_state.position).norm() / nominal_velocity;
//            std::vector<octomap::point3d> initTraj_control_points;
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
//                std::vector<octomap::point3d> target_points;
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
                        initial_traj[M - 1][i] = traj_curr[M - 1][n];
                    }
                } else {
                    initial_traj[m] = traj_curr[m + 1];
                }
            }
        }
        else{
            throw std::invalid_argument("[TrajPlanner] multisim_time_step must be equal to segment time");
        }
    }

    void TrajPlanner::initialTrajPlanningORCA(){
        updateORCAVelocity(false);

        octomap::point3d new_velocity = getAgentORCAVelocity();
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
            for(int oi = 0; oi < N_obs; oi++){
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
        for(int oi = 0; oi < N_obs; oi++){
            rvo_simulator_2d->addAgent(RVO2D::Vector2(obstacles[oi].pose.position.x,
                                                      obstacles[oi].pose.position.y));
            rvo_simulator_2d->setAgentRadius(oi + 1, obstacles[oi].radius * param.orca_inflation_ratio);
            rvo_simulator_2d->setAgentIsDynamicObstacle(oi + 1, obstacles[oi].type != ObstacleType::AGENT);
            if(isObsPredWithORCA and obstacles[oi].type == ObstacleType::AGENT){
                agentsList.emplace_back(oi + 1);
            }

            RVO2D::Vector2 obstacle_velocity = RVO2D::Vector2(obstacles[oi].velocity.linear.x,
                                                              obstacles[oi].velocity.linear.y);
            rvo_simulator_2d->setAgentVelocity(oi + 1, obstacle_velocity);

            if(isObsPredWithORCA and obstacles[oi].type == ObstacleType::AGENT){
                RVO2D::Vector2 obstacle_goal_point = RVO2D::Vector2(obstacles[oi].goal_point.x,
                                                                    obstacles[oi].goal_point.y);
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
        for(int i : agentsList){
            RVO2D::Vector2 rvo_orca_velocity = rvo_simulator_2d->getAgentVelocity(i);
            octomap::point3d orca_velocity = octomap::point3d(rvo_orca_velocity.x(), rvo_orca_velocity.y(), 0);
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
        for (int oi = 0; oi < N_obs; oi++) {
            rvo_simulator_3d->addAgent(RVO3D::Vector3(obstacles[oi].pose.position.x,
                                                      obstacles[oi].pose.position.y,
                                                      obstacles[oi].pose.position.z));
            rvo_simulator_3d->setAgentRadius(oi + 1, obstacles[oi].radius * param.orca_inflation_ratio);

            rvo_simulator_3d->setAgentIsDynamicObstacle(oi + 1, obstacles[oi].type != ObstacleType::AGENT);
            if (isObsPredWithORCA and obstacles[oi].type == ObstacleType::AGENT) {
                agentsList.emplace_back(oi + 1);
            }

            RVO3D::Vector3 obstacle_velocity = RVO3D::Vector3(obstacles[oi].velocity.linear.x,
                                                              obstacles[oi].velocity.linear.y,
                                                              obstacles[oi].velocity.linear.z);
            rvo_simulator_3d->setAgentVelocity(oi + 1, obstacle_velocity);

            if (isObsPredWithORCA) {
                RVO3D::Vector3 obstacle_goal_point = RVO3D::Vector3(obstacles[oi].goal_point.x,
                                                                    obstacles[oi].goal_point.y,
                                                                    obstacles[oi].goal_point.z);
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
            orca_velocities[qi] = octomap::point3d(orca_velocity.x(),
                                                   orca_velocity.y(),
                                                   orca_velocity.z());
        }

        // delete all agents and obstacles
        rvo_simulator_3d->deleteAllAgents();
    }

    void TrajPlanner::generateCollisionConstraints(){
        constraints.initialize(N_obs, M, n, param.dt,obs_slack_indices);

        // LSC (or BVC) construction
        ros::Time lsc_start_time = ros::Time::now();
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
        octomap::point3d normal_vector;

        for (int oi = 0; oi < N_obs; oi++) {
            octomap::point3d obstacle_point, static_obs_type;
            if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
                obstacle_point = transformStaticObstacle(oi, static_obs_type);
            }

            for (int m = 0; m < M; m++) {
                if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
                    normal_vector = normalVectorStaticObs(obstacles[oi],
                                                          initial_traj[m][0],
                                                          initial_traj[m][n],
                                                          obstacle_point,
                                                          static_obs_type);
                } else {
                    normal_vector = normalVector(obs_pred_trajs[oi][m][0],
                                                 obs_pred_trajs[oi][m][n],
                                                 initial_traj[m][0],
                                                 initial_traj[m][n],
                                                 closest_dist);
                }

                std::vector<double> d;
                d.resize(n + 1);
                octomap::point3d p_init_rel_start, p_init_rel_end;
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
                        for (int oi = 0; oi < N_obs; oi++) {
                            obs_pred_trajs[oi][m][i] = obs_pred_trajs[oi][0][0] +
                                                       (obs_pred_trajs[oi][m][i] - obs_pred_trajs[oi][0][0]) * alpha;
                        }
                    }
                }
            }
        }

        octomap::point3d normal_vector;
        for (int oi = 0; oi < N_obs; oi++) {
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

            // Static obstacle transformation (Not used in this work)
            octomap::point3d closest_obstacle_point, static_obs_type;
            if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
                closest_obstacle_point = transformStaticObstacle(oi, static_obs_type);
            }

            // Normal vector planning
            for (int m = 0; m < M; m++) {
                if(linear_prediction){
                    if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
                        normal_vector = normalVectorStaticObs(obstacles[oi],
                                                              initial_traj_trans[m][0],
                                                              initial_traj_trans[m][n],
                                                              closest_obstacle_point,
                                                              static_obs_type);
                    } else{
                        double closest_dist;
                        normal_vector = normalVector(obs_pred_traj_trans[m][0],
                                                     obs_pred_traj_trans[m][n],
                                                     initial_traj_trans[m][0],
                                                     initial_traj_trans[m][n],
                                                     closest_dist);
                    }
                } else {
                    if(obstacles[oi].type == ObstacleType::STATICOBSTACLE){
                        throw std::invalid_argument("[TrajPlanner] This work do not support, use octomap.");
                    }
                    else{
                        // Normal vector of LSC
                        normal_vector = normalVectorBetweenPolys(initial_traj_trans[m],
                                                                 obs_pred_traj_trans[m]);
                    }
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
        octomap::point3d normal_vector;
        for (int oi = 0; oi < N_obs; oi++) {
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
        if(param.planner_mode == PlannerMode::LSC){
            generateFeasibleSFC();
        }
        else{
            throw std::invalid_argument("[TrajPlanner] Current planner mode does not support SFC.");
        }
    }

    void TrajPlanner::generateFeasibleSFC(){
        CorridorConstructor corridor_constructor(distmap_obj, mission, param);

        if(flag_initialize_sfc){
            Box box = corridor_constructor.expandBoxFromPoint(agent.current_state.position,
                                                              agent.current_goal_position,
                                                              agent.radius);
            for(int m = 0; m < M; m++){
                constraints.setSFC(m, box);
            }

            flag_initialize_sfc = false;
        }
        else {
            for(int m = 1; m < M; m++){
                SFC sfc_prev = constraints.getSFC(m);
                constraints.setSFC(m-1, sfc_prev);
            }

            // update box library using last traj point
            Box box_cand;
            box_cand = corridor_constructor.expandBoxFromPoint(traj_curr[M - 1][n],
                                                               agent.current_goal_position,
                                                               agent.radius);

            //TODO: Note: Below code may cause infeasible optimization problem because the function
            //      "corridor_constructor.expandBoxFromPoint" does not guarantee that
            //      traj_curr[M - 1][n] is belong to box_cand due to the numerical error of dynamicEDT3D library.
            //      Thus, we need better SFC construction method!
            constraints.setSFC(M-1, box_cand); // may infeasible due to numerical issue...

            //TODO: If you want to avoid the numerical issue, use below code instead.
            //      It guarantees the feasibility of collision constraints but is prone to deadlock and too conservative.
//            if(not box_cand.isPointInBox(traj_curr[M - 1][n])){
//                ROS_WARN_STREAM("[TrajPlanner] Agent " << agent.id << " fail to update SFC, use previous SFC");
//            }
//            else{
//                constraints.setSFC(M-1, box_cand);
//            }
        }
    }

    octomap::point3d TrajPlanner::transformStaticObstacle(int oi, octomap::point3d& static_obs_type){
        if(obstacles[oi].type != ObstacleType::STATICOBSTACLE){
            throw std::invalid_argument("[TrajPlanner] input of transformStatic obstacle is not static obstacle");
        }


        ClosestPoints closest_points;
        closest_points = closestPointsBetweenLineSegmentAndStaticObs(initial_traj[0][0],
                                                                     initial_traj[M - 1][n],
                                                                     obstacles[oi],
                                                                     param.world_dimension,
                                                                     param.world_z_2d);
        octomap::point3d obstacle_point = closest_points.closest_point2;

        //verify static_obs_type of static obstacle
        std::array<double,3> box_min, box_max;
        box_min = {obstacles[oi].pose.position.x - obstacles[oi].dimensions[0],
                   obstacles[oi].pose.position.y - obstacles[oi].dimensions[1],
                   obstacles[oi].pose.position.z - obstacles[oi].dimensions[2]};
        box_max = {obstacles[oi].pose.position.x + obstacles[oi].dimensions[0],
                   obstacles[oi].pose.position.y + obstacles[oi].dimensions[1],
                   obstacles[oi].pose.position.z + obstacles[oi].dimensions[2]};

        static_obs_type = octomap::point3d(0, 0, 0);
        if(obstacle_point.x() < box_min[0] + SP_EPSILON_FLOAT){
            static_obs_type = static_obs_type + octomap::point3d(-1, 0, 0);
        }
        if(obstacle_point.y() < box_min[1] + SP_EPSILON_FLOAT){
            static_obs_type = static_obs_type + octomap::point3d(0, -1, 0);
        }
        if(obstacle_point.z() < box_min[2] + SP_EPSILON_FLOAT){
            static_obs_type = static_obs_type + octomap::point3d(0, 0, -1);
        }
        if(obstacle_point.x() > box_max[0] - SP_EPSILON_FLOAT){
            static_obs_type = static_obs_type + octomap::point3d(1, 0, 0);
        }
        if(obstacle_point.y() > box_max[1] - SP_EPSILON_FLOAT){
            static_obs_type = static_obs_type + octomap::point3d(0, 1, 0);
        }
        if(obstacle_point.z() > box_max[2] - SP_EPSILON_FLOAT){
            static_obs_type = static_obs_type + octomap::point3d(0, 0, 1);
        }

        obstacles[oi].radius = 0.0;
        obstacles[oi].downwash = agent.downwash;

        for(int m = 0; m < M; m++){
            for(int i = 0; i < n + 1; i++){
                obs_pred_trajs[oi][m][i] = obstacle_point;
            }
        }

        return obstacle_point;
    }

    bool TrajPlanner::trajOptimization() {
        Timer timer;

        // Solve QP problem using CPLEX
        timer.reset();
        try{
            traj_optimizer->solve(agent, constraints);
        }
        catch(...){
            // debug code
            for(int m = 0; m < M; m++){
                SFC sfc = constraints.getSFC(m);
                for(const auto& control_point : initial_traj[m]){
                    bool check1 = sfc.box.isPointInBox(control_point);
                    if(not check1){
                        ROS_ERROR("[TrajPlanner] SFC constraint is not feasible.");
                    }
                }
                for(int oi = 0; oi < N_obs; oi++){
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

        traj_curr = traj_optimizer->getTrajectory();
        current_qp_cost = traj_optimizer->getQPcost();

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
        marker.header.frame_id = "world";
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
        marker.header.frame_id = "world";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color = mission.color[agent.id];
        marker.color.a = 0.5;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

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
        marker.header.frame_id = "world";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.color.a = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        for (int oi = 0; oi < N_obs; oi++) {
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

                if(obstacles[oi].type == ObstacleType::STATICOBSTACLE){
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.1;
                    marker.scale.z = 0.1;
                }
                else{
                    marker.scale.x = 2 * obs_pred_sizes[oi][m_][i];
                    marker.scale.y = 2 * obs_pred_sizes[oi][m_][i];
                    marker.scale.z = 2 * obs_pred_sizes[oi][m_][i] * obstacles[oi].downwash;
                }

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
        dynamic_msgs::TrajectoryArray msg_obs_pred_traj_raw;
        msg_obs_pred_traj_raw.planner_seq = planner_seq;
        msg_obs_pred_traj_raw.trajectories.resize(N_obs);
        for(int oi = 0; oi < N_obs; oi++){
            msg_obs_pred_traj_raw.trajectories[oi] = trajToTrajMsg(obs_pred_trajs[oi], obstacles[oi].id, param.dt);
        }
        pub_obs_pred_traj_raw.publish(msg_obs_pred_traj_raw);
    }

    bool TrajPlanner::isDeadlock() const {
//        if(param.deadlock_mode == "velocity") {
            //If agent's velocity is lower than some threshold, then it determines agent is in deadlock
            double dist_to_goal = (agent.current_state.position - agent.desired_goal_position).norm();
            return planner_seq > param.deadlock_seq_threshold and
                   agent.current_state.velocity.norm() < param.deadlock_velocity_threshold and
                   dist_to_goal > param.goal_threshold;
//        }
//
//        if(param.deadlock_mode == "endpoint"){
//            return deadlock_start_seq > 0 and
//                   (planner_seq - deadlock_start_seq) > param.deadlock_seq_threshold and
//                   (traj_curr[M - 1][n] - deadlock_endpoint).norm() < SP_EPSILON_FLOAT and
//                   (traj_curr[M - 1][n] - agent.desired_goal_position).norm() > param.goal_threshold;
//        }
    }

    int TrajPlanner::findObstacleIdxById(int obs_id) const{
        int oi = -1;
        for(int i = 0; i < N_obs; i++){
            if(obstacles[i].id == obs_id){
                oi = i;
            }
        }
        return oi;
    }

    double TrajPlanner::computeCollisionTimeToDistmap(const octomap::point3d& start_position,
                                                      const octomap::point3d& goal_position,
                                                      double agent_radius,
                                                      double time_horizon){
        double collision_time = 0;
        bool isCollided = distmap_obj->getDistance(start_position) < agent_radius;
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
        octomap::point3d current_search_point;
        while(!isCollided and current_time < time_horizon){
            current_time += search_time_step;
            current_search_point = start_position + (goal_position - start_position) * (current_time / time_horizon);
            isCollided = distmap_obj->getDistance(current_search_point) < agent_radius;
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

        for (int oi = 0; oi < N_obs; oi++) {
            if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
                collision_time = computeCollisionTime(obstacles[oi],
                                                      initial_traj[0][0],
                                                      initial_traj[M - 1][n],
                                                      agent.radius, total_time_horizon,
                                                      param.world_dimension, param.world_z_2d);
            } else {
                collision_time = computeCollisionTime(obs_pred_trajs[oi][0][0],
                                                      obs_pred_trajs[oi][M - 1][n],
                                                      initial_traj[0][0],
                                                      initial_traj[M - 1][n],
                                                      obstacles[oi].radius + agent.radius,
                                                      total_time_horizon);
            }
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
                if (obstacles[oj].type == ObstacleType::STATICOBSTACLE) {
                    continue;
                }

                if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
                    collision_time = computeCollisionTime(obstacles[oi],
                                                          obs_pred_trajs[oj][0][0],
                                                          obs_pred_trajs[oj][M - 1][n],
                                                          obstacles[oj].radius, total_time_horizon,
                                                          param.world_dimension, param.world_z_2d);
                } else if (oj > oi) {
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

    octomap::point3d TrajPlanner::normalVector(const octomap::point3d& obs_start, const octomap::point3d& obs_goal,
                                               const octomap::point3d& agent_start, const octomap::point3d& agent_goal,
                                               double& closest_dist) {
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenLinePaths(obs_start, obs_goal, agent_start, agent_goal);
        closest_dist = closest_points.dist;

        octomap::point3d delta, normal_vector;
        delta = closest_points.closest_point2 - closest_points.closest_point1;
        normal_vector = delta.normalized();
        if (normal_vector.norm() == 0) {
            ROS_WARN("[Util] heuristic method was used to get normal vector");
            octomap::point3d a, b;
            a = agent_start - obs_start;
            b = agent_goal - obs_goal;
            if(a.norm() == 0 and b.norm() == 0){
                normal_vector = octomap::point3d(1, 0, 0);
            }
            else{
                normal_vector = (b - a).cross(octomap::point3d(0, 0, 1));
            }
        }
        return normal_vector;
    }

    octomap::point3d TrajPlanner::normalVectorGreedy(const octomap::point3d& obs_start, const octomap::point3d& obs_goal,
                                                     const octomap::point3d& agent_start, const octomap::point3d& agent_goal,
                                                     double r) {
        std::vector<octomap::point3d> n_candidates;
        octomap::point3d s, g, normal_vector, n_woIneq, n_plus, n_minus;
        s = agent_start - obs_start;
        g = agent_goal - obs_goal;

        double a, b, c, da, lambda1_plus, lambda1_minus, lambda2_plus, lambda2_minus;
        a = s.dot(s);
        b = g.dot(s);
        c = g.dot(g);
        da = a - r * r;

        n_woIneq = g.normalized();
        if(s.dot(n_woIneq) - r >= 0){
            n_candidates.emplace_back(n_woIneq);
        }
        else if (da < SP_EPSILON_FLOAT) {
            n_candidates.emplace_back(s.normalized());
        }
        else {
            lambda1_plus = (da * b + sqrt(r * r * da * (a * c - b * b))) / (a * da);
            lambda1_minus = (da * b - sqrt(r * r * da * (a * c - b * b))) / (a * da);
            lambda2_plus = (b - lambda1_plus * a) / (2 * r);
            lambda2_minus = (b - lambda1_minus * a) / (2 * r);

            if((g - s * lambda1_plus).norm() < SP_EPSILON_FLOAT or (g - s * lambda1_minus).norm() < SP_EPSILON_FLOAT){
                octomap::point3d n_z(0,0,1);
                octomap::point3d n_s = s.normalized();
                n_candidates.emplace_back(n_s * (r / s.norm()) + n_z.cross(n_s) * r * (1 - r * r / a));
            }
            else{
                n_plus = (g - s * lambda1_plus) * (1 / (2 * lambda2_plus));
                n_minus = (g - s * lambda1_minus) * (1 / (2 * lambda2_minus));
                if(n_plus.norm() != 0){
                    n_candidates.emplace_back(n_plus);
                }
                if(n_minus.norm() != 0){
                    n_candidates.emplace_back(n_minus);
                }
            }

        }

        if(n_candidates.empty()){
            throw std::invalid_argument("[TrajPlanner] n_candidates is empty");
        }

        double cost, max_cost = -SP_INFINITY;
        for(auto & n_cand : n_candidates){
            cost = g.dot(n_cand);
            if(max_cost < cost){
                normal_vector = n_cand;
                max_cost = cost;
            }
        }

        return normal_vector;
    }

    octomap::point3d TrajPlanner::normalVectorStaticObsGreedy(const octomap::point3d& obstacle_point,
                                                              const octomap::point3d& start_point,
                                                              const octomap::point3d& initial_goal_point,
                                                              const octomap::point3d& desired_goal_point,
                                                              const octomap::point3d& static_obs_type,
                                                              double radius)
    {
        octomap::point3d normal_vector, normal_vector_cand;
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenPointAndLineSegment(obstacle_point, start_point, desired_goal_point);
        if(closest_points.dist > radius){
            double closest_dist;
            normal_vector_cand = normalVector(obstacle_point, obstacle_point, start_point, desired_goal_point,
                                              closest_dist);
        }
        else{
            normal_vector_cand = normalVectorGreedy(obstacle_point, obstacle_point,
                                                    start_point, desired_goal_point,
                                                    radius);
        }

        // vertex check
        //TODO: It cannot be used at general convex obstacle
        //TODO: 3D case?
        octomap::point3d n_x, n_y, n_z;
        n_x = octomap::point3d(static_obs_type.x(), 0, 0);
        n_y = octomap::point3d(0, static_obs_type.y(), 0);
//        n_z = octomap::point3d(0, 0, static_obs_type.z());
        if(n_x.norm() > 0 and n_y.norm() > 0){
            double proj_x, proj_y;
            proj_x = normal_vector_cand.dot(n_x);
            proj_y = normal_vector_cand.dot(n_y);
            if(proj_x < 0 and proj_y < 0){
                int debug = 0;
                double closest_dist;
                normal_vector = normalVector(obstacle_point, obstacle_point, start_point, desired_goal_point,
                                             closest_dist);
            }
            if(proj_x < 0 or proj_y < 0){ //TODO: for only 2D case
                if(proj_x > proj_y){
                    normal_vector = n_x;
                }
                else{
                    normal_vector = n_y;
                }
            }
            else{
                normal_vector = normal_vector_cand;
            }
        }

        if(normal_vector.norm() == 0) {
            throw std::invalid_argument("[TrajPlanner] norm of normal vector is 0");
        }

        return normal_vector;
    }

    octomap::point3d TrajPlanner::normalVectorStaticObs(const dynamic_msgs::Obstacle &obstacle,
                                                        const octomap::point3d &pi_i_0,
                                                        const octomap::point3d &pi_i_1,
                                                        const octomap::point3d &obstacle_point,
                                                        const octomap::point3d &static_obs_type) {
        octomap::point3d normal_vector;
        if(static_obs_type.norm() > 1){
            normal_vector = normalVectorStaticObsGreedy(obstacle_point, pi_i_0, pi_i_1, agent.current_goal_position,
                                                        static_obs_type, agent.radius);
        }
        else{
            normal_vector = static_obs_type;
        }

        return normal_vector;
    }

    octomap::point3d TrajPlanner::normalVectorBetweenPolys(const std::vector<octomap::point3d> &control_points_agent,
                                                           const std::vector<octomap::point3d> &control_points_obs) {
        size_t n_control_points = control_points_agent.size();
        std::vector<octomap::point3d> control_points_rel;
        control_points_rel.resize(n_control_points);
        for(size_t i = 0; i < n_control_points; i++){
            control_points_rel[i] = control_points_agent[i] - control_points_obs[i];
        }

        ClosestPoints closest_points = closestPointsBetweenPointAndConvexHull(octomap::point3d(0,0,0),
                                                                              control_points_rel);
        octomap::point3d normal_vector = closest_points.closest_point2.normalized();
        return normal_vector;
    }
}