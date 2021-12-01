#include <multi_sync_simulator.hpp>

namespace DynamicPlanning{
    MultiSyncSimulator::MultiSyncSimulator(const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission)
            : nh(_nh), param(_param), mission(_mission), obstacle_generator(ObstacleGenerator(_nh, _mission))
    {
        pub_agent_trajectories = nh.advertise<visualization_msgs::MarkerArray>("/agent_trajectories_history", 1);
        pub_obstacle_trajectories = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_trajectories_history", 1);
        pub_collision_model = nh.advertise<visualization_msgs::MarkerArray>("/collision_model", 1);
        pub_agent_velocities_x = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_x", 1);
        pub_agent_velocities_y = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_y", 1);
        pub_agent_velocities_z = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_z", 1);
        pub_agent_accelerations_x = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_x", 1);
        pub_agent_accelerations_y = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_y", 1);
        pub_agent_accelerations_z = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_z", 1);
        pub_agent_vel_limits = nh.advertise<std_msgs::Float64MultiArray>("/agent_vel_limits", 1);
        pub_agent_acc_limits = nh.advertise<std_msgs::Float64MultiArray>("/agent_acc_limits", 1);
        pub_start_goal_points_vis = nh.advertise<visualization_msgs::MarkerArray>("/start_goal_points", 1);
        pub_goal_positions_raw = nh.advertise<dynamic_msgs::GoalArray>("/goal_positions_raw", 1);
        pub_world_boundary = nh.advertise<visualization_msgs::MarkerArray>("/world_boundary", 1);
        pub_collision_alert = nh.advertise<visualization_msgs::MarkerArray>("/collision_alert", 1);
        pub_desired_trajs_raw = nh.advertise<dynamic_msgs::TrajectoryArray>("/desired_trajs_raw", 1);
        pub_desired_trajs_vis = nh.advertise<visualization_msgs::MarkerArray>("/desired_trajs_vis", 1);
        pub_grid_map = nh.advertise<visualization_msgs::MarkerArray>("/grid_map", 1);
//        sub_octomap = nh.subscribe( "/octomap_full", 1, &MultiSyncSimulator::octomapCallback, this);
        service_start_planning = nh.advertiseService("/start_planning", &MultiSyncSimulator::startPlanningCallback, this);
        service_start_patrol = nh.advertiseService("/start_patrol", &MultiSyncSimulator::startPatrolCallback, this);
        service_stop_patrol = nh.advertiseService("/stop_patrol", &MultiSyncSimulator::stopPatrolCallback, this);
        service_update_goal = nh.advertiseService("/update_goal", &MultiSyncSimulator::updateGoalCallback, this);

        msg_agent_trajectories.markers.clear();
        msg_agent_trajectories.markers.resize(mission.qn);
        msg_obstacle_trajectories.markers.clear();
        msg_obstacle_trajectories.markers.resize(mission.on);

        sim_start_time = ros::Time::now();
        sim_current_time = sim_start_time;
        obstacle_generator.update(0, 0);

        is_collided = false;
        has_distmap = false;
        initial_update = true;
        mission_changed = true;
        safety_ratio_agent = SP_INFINITY;
        safety_ratio_obs = SP_INFINITY;
        total_flight_time = SP_INFINITY;
        total_distance = 0;
        N_average = 0;
        total_distance = 0;

        file_name_time = std::to_string(sim_start_time.toSec());
        file_name_param = param.getPlannerModeStr() + "_" + std::to_string(mission.qn) + "agents";
//        mission.saveMission(param.package_path + "/missions/previous_missions/mission_" + file_name_time + ".json");

        // Planner state intialization
        if(param.multisim_experiment){
            planner_state = PlannerState::WAIT;
        }
        else {
            // Do not wait for start signal in simulation
            if(param.multisim_patrol){
                planner_state = PlannerState::PATROL;
            }
            else{
                planner_state = PlannerState::GOTO;
            }
        }

        if(param.world_use_octomap){
            setDistmap(mission.world_file_name);
        }

        agents.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++){
            agents[qi] = std::make_unique<TrajPlanner>(qi, nh, param, mission, distmap_ptr);
            dynamic_msgs::State initial_state;
            initial_state.pose.position = point3DToPointMsg(mission.agents[qi].start_position);
            agents[qi]->setCurrentState(initial_state);
        }
    }

    void MultiSyncSimulator::run() {
        // Main Loop
        for (int iter = 0;
             (iter < param.multisim_max_planner_iteration or param.multisim_experiment) and ros::ok(); iter++) {
            ros::spinOnce();

            // Wait until map is loaded and start signal is arrived
            if(not isPlannerReady()) {
                ROS_INFO_ONCE("[MultiSyncSimulator] Planner ready, wait for message");
                ros::Rate(10).sleep();
                iter--;
                continue;
            }

            // Check mission finished
            if (isFinished() or (not param.multisim_experiment and iter == param.multisim_max_planner_iteration - 1)) {
                // Save result in csv file
                summarizeResult();

                if(param.multisim_experiment){
                    // Wait for another mission
                    planner_state = PlannerState::WAIT;
                    initial_update = true;
                    continue;
                }
                else{
                    // Planning finished
                    break;
                }
            }

            if(initial_update){
                initializeTimer();
            } else if (param.multisim_experiment and (sim_current_time - ros::Time::now()).toSec() > 0) {
                // Wait until next planning period
                // Note: planning start time = traj_start_time - multisim_time_step
                continue;
            } else {
                doStep();
            }

            // Update and broadcast agent and obstacle state
            update();

            // Trajectory planning
            bool success = plan();
            if (not success) {
                break;
            }

            // Publish planning result
            publish();

            // (For experiment) Check planning is finished in time
            // (For simulation) Slow down iteration speed for visualization
            if (param.multisim_experiment) {
                double timing_margin = (sim_current_time - ros::Time::now()).toSec();
                if (timing_margin < 0) {
                    ROS_WARN_STREAM("[MultiSyncSimulator] Planning speed is too slow! Delay time: " << timing_margin);
                }
            } else if (param.multisim_planning_rate > 0) {
                ros::Rate(param.multisim_planning_rate).sleep();
            }
        }
    }

    void MultiSyncSimulator::setCurrentState(int qi, const dynamic_msgs::State& state){
        agents[qi]->setCurrentState(state);
    }

    void MultiSyncSimulator::setDistmap(const std::string& file_name){
        float max_dist = 1.0; //TODO: parameterization
        octomap::OcTree* octree_ptr;
        octree_ptr = new octomap::OcTree(param.world_resolution);
        if(!octree_ptr->readBinary(file_name)){
            throw std::invalid_argument("[MultiSyncSimulator] Fail to read octomap file.");
        }

        distmap_ptr = std::make_shared<DynamicEDTOctomap>(max_dist, octree_ptr, mission.world_min, mission.world_max,
                                                          false);
        distmap_ptr->update();

        has_distmap = true;
    }

    bool MultiSyncSimulator::isPlannerReady() {
        bool is_planner_ready = planner_state > PlannerState::WAIT;

        if (param.world_use_octomap) {
            is_planner_ready = is_planner_ready and has_distmap;
        }

        return is_planner_ready;
    }

    void MultiSyncSimulator::initializeTimer(){
        // Initialize planner timing
        // To match the simulation time and real world time, add one time step to simulation time.
        sim_start_time = ros::Time::now() + ros::Duration(param.multisim_time_step);
        sim_current_time = sim_start_time;
    }

    void MultiSyncSimulator::doStep(){
        sim_current_time += ros::Duration(param.multisim_time_step);
    }

    void MultiSyncSimulator::update(){
        // Compute ideal state of agents
        std::vector<dynamic_msgs::State> ideal_agent_curr_states;
        std::vector<dynamic_msgs::State> ideal_agent_next_states;
        ideal_agent_curr_states.resize(mission.qn);
        ideal_agent_next_states.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++) {
            ideal_agent_curr_states[qi] = agents[qi]->getCurrentStateMsg();
            if(initial_update){
                // initial update
                ideal_agent_next_states[qi] = agents[qi]->getCurrentStateMsg();
            }
            else{
                ideal_agent_next_states[qi] = agents[qi]->getFutureStateMsg(param.multisim_time_step);
            }
        }

        // Measure current state of objects
        points_t real_agent_curr_positions;
        real_agent_curr_positions.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++) {
            bool external_pose_update = true;
            tf::StampedTransform transform;
            try {
                tf_listener.lookupTransform(param.world_frame_id, "/cf" + std::to_string(mission.agents[qi].cid),
                                            ros::Time(0), transform);
                real_agent_curr_positions[qi] = vector3MsgToPoint3d(transform.getOrigin());
            }
            catch (tf::TransformException &ex) {
                ROS_WARN_ONCE("[MultiSyncSimulator] tf listener failed, use ideal state instead.");
                external_pose_update = false;
            }

            if(not external_pose_update){
                // if there is no external pose update, then use ideal agent state.
                real_agent_curr_positions[qi] = pointMsgToPoint3d(ideal_agent_curr_states[qi].pose.position);
            }
        }

        // Update agent's states
        // If difference between ideal and observed position is small, then use ideal position
        for(int qi = 0; qi < mission.qn; qi++) {
            point_t ideal_agent_curr_position = pointMsgToPoint3d(ideal_agent_curr_states[qi].pose.position);
            double diff = (ideal_agent_curr_position - real_agent_curr_positions[qi]).norm();
            if (diff > param.multisim_reset_threshold) {
                dynamic_msgs::State current_state = ideal_agent_curr_states[qi];
                current_state.pose.position = point3DToPointMsg(real_agent_curr_positions[qi]);
                current_state.pose.orientation = defaultQuaternion();
                current_state.velocity.linear = defaultVector();
                current_state.acceleration.linear = defaultVector();
                agents[qi]->setCurrentState(current_state);
                ROS_WARN_STREAM("[MultiSyncSimulator] agent " << qi
                                                          << ": diff between ideal and real is too big diff=" << diff);
            } else {
                agents[qi]->setCurrentState(ideal_agent_next_states[qi]);
            }
        }

        // Update obstacle's states to each agent
        for(int qi = 0; qi < mission.qn; qi++){
            dynamic_msgs::ObstacleArray msg_obstacles;
            std::vector<traj_t> obs_prev_trajs;
            msg_obstacles.obstacles.clear();
            msg_obstacles.start_time = sim_start_time;
            msg_obstacles.header.stamp = sim_current_time;

            //TODO: Dynamic obstacles
//            // Update dynamic obstacles (only for reciprocal_rsfc)
//            if(param.constraint_mode == "reciprocal_rsfc") {
//                obstacle_generator.update((sim_current_time - sim_start_time).toSec(), param.multisim_observer_stddev);
//                for (int oi = 0; oi < mission.on; oi++) {
//                    dynamic_msgs::Obstacle obstacle = obstacle_generator.getObstacle(oi);
//                    traj_t empty_traj;
//                    msg_obstacles.obstacles.emplace_back(obstacle);
//                    obs_prev_trajs.emplace_back(empty_traj);
//                }
//            }

            // Other agents
            for(int qj = 0; qj < mission.qn; qj++){
                if(qi == qj){
                    continue;
                }

                dynamic_msgs::Obstacle obstacle;
                obstacle.id = qj;
                obstacle.type = ObstacleType::AGENT;

                // If diff between ideal and observed position is small, then use ideal position
                point_t ideal_agent_curr_position = pointMsgToPoint3d(ideal_agent_curr_states[qj].pose.position);
                if((ideal_agent_curr_position - real_agent_curr_positions[qj]).norm() > param.multisim_reset_threshold)
                {
                    obstacle.pose.position = point3DToPointMsg(real_agent_curr_positions[qj]);
                    obstacle.pose.orientation = defaultQuaternion();
                    obstacle.velocity.linear = defaultVector();
                }
                else {
                    obstacle.pose = ideal_agent_next_states[qj].pose;
                    obstacle.velocity = ideal_agent_next_states[qj].velocity;
                }

                obstacle.goal = point3DToPointMsg(agents[qj]->getDesiredGoalPosition());
                obstacle.radius = mission.agents[qj].radius;
                obstacle.max_acc = mission.agents[qj].max_acc[0];
                obstacle.downwash = mission.agents[qj].downwash;
                msg_obstacles.obstacles.emplace_back(obstacle);

                traj_t obs_traj = agents[qj]->getTraj();
                obs_prev_trajs.emplace_back(obs_traj);
            }

            agents[qi]->setObstacles(msg_obstacles);
            agents[qi]->setObsPrevTrajs(obs_prev_trajs);
            agents[qi]->setPlannerState(planner_state);

            if(mission_changed){
                agents[qi]->setStartPosition(mission.agents[qi].start_position);
                agents[qi]->setDesiredGoal(mission.agents[qi].desired_goal_position);
            }
        }

        if(initial_update){
            initial_update = false;
        }
        if(mission_changed){
            mission_changed = false;
        }
    }

    bool MultiSyncSimulator::plan() {
        // Sequential planning
        PlanningReport result;
        for(int qi = 0; qi < mission.qn; qi++){
            result = agents[qi]->plan(sim_current_time);
            if(result == PlanningReport::QPFAILED){
                return false;
            }
        }

        // save planning result
        savePlanningResult();
        if(param.multisim_save_result){
            savePlanningResultAsCSV();
        }

        return true;
    }

    void MultiSyncSimulator::publish(){
        publishAgentTrajectories();
        publishCollisionModel();
        publishStartGoalPoints();
        publishWorldBoundary();
        publishCollisionAlert();
        for(int qi = 0; qi < mission.qn; qi++){
            agents[qi]->publish();
            obstacle_generator.publishForAgent(qi);
        }
        obstacle_generator.publish();
        publishAgentState();
        publishDesiredTrajs();

//        publishGridMap();
    }

    bool MultiSyncSimulator::isFinished(){
        if(planner_state == PlannerState::PATROL){
            return false;
        }

        for(int qi = 0; qi < mission.qn; qi++) {
            point_t current_position = agents[qi]->getCurrentPosition();
            double dist_to_goal;
            if(planner_state == PlannerState::GOTO){
                dist_to_goal = (current_position - mission.agents[qi].desired_goal_position).norm();
            }
            else if(planner_state == PlannerState::GOBACK){
                dist_to_goal = (current_position - mission.agents[qi].start_position).norm();
            }

            if (dist_to_goal > param.goal_threshold) {
                return false;
            }
        }

        total_flight_time = (sim_current_time - sim_start_time).toSec();
        return true;
    }

    void MultiSyncSimulator::summarizeResult(){
        // total_flight_time
        ROS_INFO_STREAM("[MultiSyncSimulator] total flight time: " << total_flight_time);

        // total flight distance
        total_distance = getTotalDistance();
        ROS_INFO_STREAM("[MultiSyncSimulator] total disance: " << total_distance);

        // average planning time
        ROS_INFO_STREAM("[MultiSyncSimulator] planning time per agent: " << planning_time.total_planning_time.average);

        // safety_ratio_agent
        ROS_INFO_STREAM("[MultiSyncSimulator] safety ratio between agent: " << safety_ratio_agent);

        // safety_ratio_obstacle
//        ROS_INFO_STREAM("[MultiSyncSimulator] safety ratio obstacle: " << safety_ratio_obs);

        if(param.multisim_save_result){
            saveSummarizedResultAsCSV();
        }
    }

    void MultiSyncSimulator::setObstacles(int qi, const dynamic_msgs::ObstacleArray& obstacles){
        agents[qi]->setObstacles(obstacles);
    }

    void MultiSyncSimulator::savePlanningResult(){
        double record_time_step = param.multisim_record_time_step;
        double future_time = 0;
        while(future_time < param.multisim_time_step - SP_EPSILON_FLOAT){
            for(int qi = 0; qi < mission.qn; qi++){
                msg_agent_trajectories.markers[qi].header.frame_id = param.world_frame_id;
                msg_agent_trajectories.markers[qi].type = visualization_msgs::Marker::LINE_STRIP;
                msg_agent_trajectories.markers[qi].action = visualization_msgs::Marker::ADD;
                msg_agent_trajectories.markers[qi].ns = std::to_string(qi);
                msg_agent_trajectories.markers[qi].id = qi;
                msg_agent_trajectories.markers[qi].scale.x = 0.03;
                msg_agent_trajectories.markers[qi].pose.position = defaultPoint();
                msg_agent_trajectories.markers[qi].pose.orientation = defaultQuaternion();
                msg_agent_trajectories.markers[qi].color = mission.color[qi];
                msg_agent_trajectories.markers[qi].color.a = 0.5;
                dynamic_msgs::State future_state = agents[qi]->getFutureStateMsg(future_time);
                msg_agent_trajectories.markers[qi].points.emplace_back(future_state.pose.position);
            }

            for(int oi = 0; oi < mission.on; oi++){
                msg_obstacle_trajectories.markers[oi].header.frame_id = param.world_frame_id;
                msg_obstacle_trajectories.markers[oi].type = visualization_msgs::Marker::LINE_STRIP;
                msg_obstacle_trajectories.markers[oi].action = visualization_msgs::Marker::ADD;
                msg_obstacle_trajectories.markers[oi].scale.x = 0.05;
                msg_obstacle_trajectories.markers[oi].id = oi;

                msg_obstacle_trajectories.markers[oi].color.r = 0;
                msg_obstacle_trajectories.markers[oi].color.g = 0;
                msg_obstacle_trajectories.markers[oi].color.b = 0;
                msg_obstacle_trajectories.markers[oi].color.a = 1;

                dynamic_msgs::Obstacle obstacle = obstacle_generator.getObstacle(oi);  //TODO: fix this part!
                msg_obstacle_trajectories.markers[oi].points.emplace_back(obstacle.pose.position);
            }

            future_time += record_time_step;
        }

        // minimum distance
        future_time = 0;
        while(future_time < param.multisim_time_step - SP_EPSILON_FLOAT) {
            for (int qi = 0; qi < mission.qn; qi++) {
                dynamic_msgs::State agent_state_i = agents[qi]->getFutureStateMsg(future_time);
                point_t agent_position_i = pointMsgToPoint3d(agent_state_i.pose.position);
                double current_safety_ratio_agent = SP_INFINITY;
                int min_qj = -1;
                for (int qj = 0; qj < mission.qn; qj++) {
                    if (qi == qj) {
                        continue;
                    }

                    double downwash = (mission.agents[qi].downwash * mission.agents[qi].radius + mission.agents[qj].downwash * mission.agents[qj].radius) / (mission.agents[qi].radius + mission.agents[qj].radius);
                    dynamic_msgs::State agent_state_j = agents[qj]->getFutureStateMsg(future_time);
                    point_t agent_position_j = pointMsgToPoint3d(agent_state_j.pose.position);
                    double dist_to_agent = ellipsoidalDistance(agent_position_i, agent_position_j, downwash);

                    double safety_ratio = dist_to_agent / (mission.agents[qi].radius + mission.agents[qj].radius);
                    if (safety_ratio < current_safety_ratio_agent) {
                        current_safety_ratio_agent = safety_ratio;
                        min_qj = qj;
                    }
                    if (safety_ratio < safety_ratio_agent) {
                        safety_ratio_agent = safety_ratio;
                    }
                }
                if (current_safety_ratio_agent < 1) {
                    ROS_ERROR_STREAM("[MultiSyncSimulator] collision with agents, agent_id: (" << qi << "," << min_qj
                                                                                               << "), safety_ratio:"
                                                                                               << current_safety_ratio_agent);
                    is_collided = true;
                }

                double current_safety_margin_obs = SP_INFINITY;
                for (int oi = 0; oi < mission.on; oi++) {
                    dynamic_msgs::Obstacle obstacle = obstacle_generator.getObstacle(oi);
                    point_t obs_position = pointMsgToPoint3d(obstacle.pose.position);
                    double dist_to_obs = (agent_position_i - obs_position).norm();

                    double safety_ratio = dist_to_obs / (mission.agents[qi].radius + obstacle.radius);
                    if (safety_ratio < current_safety_margin_obs) {
                        current_safety_margin_obs = safety_ratio;
                    }
                    if (safety_ratio < safety_ratio_obs) {
                        safety_ratio_obs = safety_ratio;
                    }
                }
                if (current_safety_margin_obs < 1) {
                    ROS_ERROR_STREAM(
                            "[MultiSyncSimulator] collision with obstacles, agent_id:" << qi
                                                                                   << ", current_safety_ratio:"
                                                                                   << current_safety_margin_obs);
                    is_collided = true;
                }
            }
            future_time += record_time_step;
        }

        // planning time
        for(int qi = 0; qi < mission.qn; qi++){
            N_average++;
            PlanningTimeStatistics agent_planning_time = agents[qi]->getPlanningTime();
            planning_time.update(agent_planning_time);
        }
    }

    void MultiSyncSimulator::savePlanningResultAsCSV(){
        std::string file_name = param.package_path + "/log/result_" + file_name_time + "_" + file_name_param + ".csv";
        std::ofstream result_csv;
        result_csv.open(file_name, std::ios_base::app);
        if (sim_current_time == sim_start_time) {
            for(int qi = 0; qi < mission.qn; qi++){
                result_csv << "id,t,px,py,pz,vx,vy,vz,ax,ay,az,planning_time,qp_cost,planning_report,size";
                if(qi < mission.qn - 1 or mission.on != 0){
                    result_csv << ",";
                }
                else{
                    result_csv << "\n";
                }
            }

            for(int oi = 0; oi < mission.on; oi++){
                result_csv << "obs_id,t,px,py,pz,size";
                if(oi < mission.on - 1){
                    result_csv << ",";
                }
                else{
                    result_csv << "\n";
                }
            }
        }

        double record_time_step = param.multisim_record_time_step;
        double future_time = 0;
        double t = (sim_current_time - sim_start_time).toSec();
        while(future_time < param.multisim_time_step) {
            for (int qi = 0; qi < mission.qn; qi++) {
                result_csv << qi << "," << t << ","
                        << agents[qi]->getFutureStateMsg(future_time).pose.position.x << ","
                        << agents[qi]->getFutureStateMsg(future_time).pose.position.y << ","
                        << agents[qi]->getFutureStateMsg(future_time).pose.position.z << ","
                        << agents[qi]->getFutureStateMsg(future_time).velocity.linear.x << ","
                        << agents[qi]->getFutureStateMsg(future_time).velocity.linear.y << ","
                        << agents[qi]->getFutureStateMsg(future_time).velocity.linear.z << ","
                        << agents[qi]->getFutureStateMsg(future_time).acceleration.linear.x << ","
                        << agents[qi]->getFutureStateMsg(future_time).acceleration.linear.y << ","
                        << agents[qi]->getFutureStateMsg(future_time).acceleration.linear.z << ","
                        << agents[qi]->getPlanningTime().total_planning_time.current << ","
                        << agents[qi]->getTrajCost() << ","
                           << agents[qi]->getPlanningReport() << ","
                           << mission.agents[qi].radius;

                if (qi < mission.qn - 1 or mission.on != 0) {
                    result_csv << ",";
                } else {
                    result_csv << "\n";
                }
            }


            for(int oi = 0; oi < mission.on; oi++) {
                dynamic_msgs::Obstacle obstacle = obstacle_generator.getObstacle(oi);  //TODO: fix this part!
                result_csv << oi << "," << t << ","
                           << obstacle.pose.position.x << ","
                           << obstacle.pose.position.y << ","
                           << obstacle.pose.position.z << ","
                           << obstacle.radius;
                if(oi < mission.on - 1){
                    result_csv << ",";
                }
                else{
                    result_csv << "\n";
                }
            }

            future_time += record_time_step;
            t += record_time_step;
        }

        result_csv.close();
    }

    void MultiSyncSimulator::saveSummarizedResultAsCSV(){
        std::string file_name = param.package_path + "/log/summary_" + file_name_param + ".csv";
        std::ifstream result_csv_in(file_name);
        bool print_description = false;
        if(not result_csv_in or result_csv_in.peek() == std::ifstream::traits_type::eof()){
            print_description = true;
        }

        std::ofstream result_csv_out;
        result_csv_out.open(file_name, std::ios_base::app);
        if(print_description){
            result_csv_out << "start_time,total_flight_time,total_flight_distance,is_collided,safety_ratio_agent,"
                           << "average_planning_time,min_planning_time,max_planning_time,"
                           << "initial_traj_planning_time,obstacle_prediction_time,goal_planning_time,"
                           << "lsc_generation_time,sfc_generation_time,traj_optimization_time,"
                           << "mission_file_name,world_file_name,planner_mode,prediction_mode,initial_traj_mode,"
                           << "slack_mode,goal_mode,world_dimension,M,dt,N_constraint_segments\n";
        }
        result_csv_out << sim_start_time << ","
                       << total_flight_time << ","
                       << total_distance << ","
                       << is_collided << ","
                       << safety_ratio_agent << ","
                       << planning_time.total_planning_time.average << ","
                       << planning_time.total_planning_time.min << ","
                       << planning_time.total_planning_time.max << ","
                       << planning_time.initial_traj_planning_time.average << ","
                       << planning_time.obstacle_prediction_time.average << ","
                       << planning_time.goal_planning_time.average << ","
                       << planning_time.lsc_generation_time.average << ","
                       << planning_time.sfc_generation_time.average << ","
                       << planning_time.traj_optimization_time.average << ","
                       << mission.mission_file_name << ","
                       << mission.world_file_name << ","
                       << param.getPlannerModeStr() << ","
                       << param.getPredictionModeStr() << ","
                       << param.getInitialTrajModeStr() << ","
                       << param.getSlackModeStr() << ","
                       << param.getGoalModeStr() << ","
                       << param.world_dimension << ","
                       << param.M << ","
                       << param.dt << ","
                       << param.N_constraint_segments << "\n";
        result_csv_out.close();
    }

    void MultiSyncSimulator::saveNormalVectorAsCSV(){
        std::string file_name = param.package_path + "/log/normalSum_" + std::to_string(sim_start_time.toSec())
                                + "_" + param.getPlannerModeStr() + "_" + std::to_string(mission.qn) + "agents.csv";
        std::ofstream normal_csv;
        normal_csv.open(file_name, std::ios_base::app);

        if (sim_current_time == sim_start_time) {
            normal_csv << "t,";
            for (int qi = 0; qi < mission.qn; qi++) {
                for (int qj = qi + 1; qj < mission.qn; qj++) {
                    normal_csv << "(" << qi << "-" << qj << ")";
                    if (qi == mission.qn - 2) {
                        normal_csv << "\n";
                    } else {
                        normal_csv << ",";
                    }
                }
            }
        }

        double t = (sim_current_time - sim_start_time).toSec();
        normal_csv << t << ",";
        for (int qi = 0; qi < mission.qn; qi++) {
            for (int qj = qi + 1; qj < mission.qn; qj++) {
                point_t normal_vector = agents[qi]->getNormalVector(qj, 0) + agents[qj]->getNormalVector(qi, 0);
                normal_csv << normal_vector.x();
                if(qi == mission.qn - 2){
                    normal_csv << "\n";
                }
                else{
                    normal_csv << ",";
                }
            }
        }
    }

    double MultiSyncSimulator::getTotalDistance(){
        double dist_total = 0;
        for(int qi = 0; qi < mission.qn; qi++){
            for(int i = 0; i < (int)msg_agent_trajectories.markers[qi].points.size() - 1; i++) {
                dist_total += (pointMsgToPoint3d(msg_agent_trajectories.markers[qi].points[i + 1]) -
                               pointMsgToPoint3d(msg_agent_trajectories.markers[qi].points[i])).norm();
            }
        }
        return dist_total;
    }

    void MultiSyncSimulator::octomapCallback(const octomap_msgs::Octomap& octomap_msg){
        if(has_distmap){
            return;
        }

        float max_dist = 1.0; //TODO: parameterization
        auto* octree_ptr = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg));
        distmap_ptr = std::make_shared<DynamicEDTOctomap>(max_dist, octree_ptr,
                                                          mission.world_min, mission.world_max,
                                                          false);
        distmap_ptr->update();
        has_distmap = true;
    }

    bool MultiSyncSimulator::startPlanningCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
        planner_state = PlannerState::GOTO;
        return true;
    }

    bool MultiSyncSimulator::startPatrolCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
        planner_state = PlannerState::PATROL;
        return true;
    }

    bool MultiSyncSimulator::stopPatrolCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
        planner_state = GOBACK;
        return true;
    }

    bool MultiSyncSimulator::updateGoalCallback(dynamic_msgs::UpdateGoals::Request& req,
                                                dynamic_msgs::UpdateGoals::Response& res){

        bool success = (not req.file_name.empty()) and mission.initialize(req.file_name,
                                                                          param.multisim_max_noise,
                                                                          param.world_dimension,
                                                                          param.world_z_2d);
        if(success){
            ROS_INFO("Update goal success");
            mission_changed = true;
        }
        else{
            ROS_ERROR("Failed to update goal, invalid filename");
            return false;
        }

        return true;
    }

    void MultiSyncSimulator::publishCollisionModel(){
        visualization_msgs::MarkerArray msg_collision_model;
        msg_collision_model.markers.clear();

        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        for(int qi = 0; qi < mission.qn; qi++) {
            marker.color = mission.color[qi];
            marker.color.a = 0.3;

            marker.scale.x = 2 * mission.agents[qi].radius;
            marker.scale.y = 2 * mission.agents[qi].radius;
            marker.scale.z = 2 * mission.agents[qi].radius * mission.agents[qi].downwash;

            marker.id = qi;
            dynamic_msgs::State current_state = agents[qi]->getCurrentStateMsg();
            marker.pose.position = current_state.pose.position;
            marker.pose.orientation = defaultQuaternion();

            msg_collision_model.markers.emplace_back(marker);
        }
        pub_collision_model.publish(msg_collision_model);
    }

    void MultiSyncSimulator::publishStartGoalPoints(){
        dynamic_msgs::GoalArray msg_goal_positions_raw;
        visualization_msgs::MarkerArray msg_start_goal_points_vis;

        for(int qi = 0; qi < mission.qn; qi++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = param.world_frame_id;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color = mission.color[qi];
            marker.color.a = 0.7;

            marker.ns = "start";
            marker.id = qi;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.pose.position = point3DToPointMsg(mission.agents[qi].start_position);
            marker.pose.orientation = point3DToQuaternionMsg(point_t(0,0,0));
            msg_start_goal_points_vis.markers.emplace_back(marker);

            point_t agent_desired_goal = agents[qi]->getDesiredGoalPosition();
            marker.ns = "desired_goal";
            marker.id = qi;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.pose.position = point3DToPointMsg(agent_desired_goal);
            marker.pose.orientation = defaultQuaternion();
            msg_start_goal_points_vis.markers.emplace_back(marker);

            point_t agent_current_goal = agents[qi]->getCurrentGoalPosition();
            marker.ns = "current_goal";
            marker.id = qi;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.pose.position = point3DToPointMsg(agent_current_goal);
            marker.pose.orientation = point3DToQuaternionMsg(point_t(0,0,0));
            msg_start_goal_points_vis.markers.emplace_back(marker);

            dynamic_msgs::Goal goal;
            goal.planner_seq = agents[qi]->getPlannerSeq();
            goal.id = qi;
            goal.current_goal = point3DToPointMsg(agent_current_goal);
            goal.desired_goal = point3DToPointMsg(agent_desired_goal);
            msg_goal_positions_raw.goals.emplace_back(goal);
        }
        pub_start_goal_points_vis.publish(msg_start_goal_points_vis);
        pub_goal_positions_raw.publish(msg_goal_positions_raw);
    }

    void MultiSyncSimulator::publishWorldBoundary(){
        visualization_msgs::MarkerArray msg_world_boundary;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        std::vector<double> world_boundary{mission.world_min.x(), mission.world_min.y(), mission.world_min.z(),
                                           mission.world_max.x(), mission.world_max.y(), mission.world_max.z()};

        marker.pose.position = defaultPoint();
        marker.pose.orientation = defaultQuaternion();

        marker.scale.x = 0.03;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        geometry_msgs::Point point_i, point_j;
        std::vector<int> index = {0, 1, 2,
                                  3, 4, 2,
                                  3, 1, 5,
                                  0, 4, 5};
        int offset = 0;
        for (int iter = 0; iter < 4; iter++) {
            point_i.x = world_boundary[index[offset + 0]];
            point_i.y = world_boundary[index[offset + 1]];
            point_i.z = world_boundary[index[offset + 2]];

            for (int i = 0; i < 3; i++) {
                point_j.x = world_boundary[(index[offset + 0] + 3 * (i == 0)) % 6];
                point_j.y = world_boundary[(index[offset + 1] + 3 * (i == 1)) % 6];
                point_j.z = world_boundary[(index[offset + 2] + 3 * (i == 2)) % 6];

                marker.points.emplace_back(point_i);
                marker.points.emplace_back(point_j);
            }
            offset += 3;
        }
        msg_world_boundary.markers.emplace_back(marker);
        pub_world_boundary.publish(msg_world_boundary);
    }

    void MultiSyncSimulator::publishAgentTrajectories(){
        pub_agent_trajectories.publish(msg_agent_trajectories);
    }

    void MultiSyncSimulator::publishCollisionAlert(){
        visualization_msgs::MarkerArray msg_collision_alert;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position = point3DToPointMsg((mission.world_max + mission.world_min) * 0.5);
        marker.pose.orientation = defaultQuaternion();

        marker.scale.x = mission.world_max.x() - mission.world_min.x();
        marker.scale.y = mission.world_max.y() - mission.world_min.y();
        marker.scale.z = mission.world_max.z() - mission.world_min.z();

        marker.color.a = 0.0;
        if(safety_ratio_agent < 1 or safety_ratio_obs < 1){
            marker.color.a = 0.3;
        }
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;

        msg_collision_alert.markers.emplace_back(marker);
        pub_collision_alert.publish(msg_collision_alert);
    }

    void MultiSyncSimulator::publishAgentState(){
        std_msgs::Float64MultiArray msg_agent_velocities_x;
        std_msgs::Float64MultiArray msg_agent_velocities_y;
        std_msgs::Float64MultiArray msg_agent_velocities_z;
        std_msgs::Float64MultiArray msg_agent_acceleration_x;
        std_msgs::Float64MultiArray msg_agent_acceleration_y;
        std_msgs::Float64MultiArray msg_agent_acceleration_z;
        std_msgs::Float64MultiArray msg_agent_vel_limits;
        std_msgs::Float64MultiArray msg_agent_acc_limits;

        for(int qi = 0; qi < mission.qn; qi++){
            msg_agent_velocities_x.data.emplace_back(agents[qi]->getCurrentStateMsg().velocity.linear.x);
            msg_agent_velocities_y.data.emplace_back(agents[qi]->getCurrentStateMsg().velocity.linear.y);
            msg_agent_velocities_z.data.emplace_back(agents[qi]->getCurrentStateMsg().velocity.linear.z);
            msg_agent_acceleration_x.data.emplace_back(agents[qi]->getCurrentStateMsg().acceleration.linear.x);
            msg_agent_acceleration_y.data.emplace_back(agents[qi]->getCurrentStateMsg().acceleration.linear.y);
            msg_agent_acceleration_z.data.emplace_back(agents[qi]->getCurrentStateMsg().acceleration.linear.z);
        }
        msg_agent_vel_limits.data.emplace_back(mission.agents[0].max_vel[0]);
        msg_agent_vel_limits.data.emplace_back(-mission.agents[0].max_vel[0]);
        msg_agent_acc_limits.data.emplace_back(mission.agents[0].max_acc[0]);
        msg_agent_acc_limits.data.emplace_back(-mission.agents[0].max_acc[0]);

        pub_agent_velocities_x.publish(msg_agent_velocities_x);
        pub_agent_velocities_y.publish(msg_agent_velocities_y);
        pub_agent_velocities_z.publish(msg_agent_velocities_z);
        pub_agent_accelerations_x.publish(msg_agent_acceleration_x);
        pub_agent_accelerations_y.publish(msg_agent_acceleration_y);
        pub_agent_accelerations_z.publish(msg_agent_acceleration_z);
        pub_agent_vel_limits.publish(msg_agent_vel_limits);
        pub_agent_acc_limits.publish(msg_agent_acc_limits);
    }
    void MultiSyncSimulator::publishDesiredTrajs() {
        // Raw
        dynamic_msgs::TrajectoryArray msg_desired_trajs_raw;
        msg_desired_trajs_raw.header.stamp = sim_current_time;
        msg_desired_trajs_raw.planner_seq = agents[0]->getPlannerSeq();
        msg_desired_trajs_raw.trajectories.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++){
            msg_desired_trajs_raw.trajectories[qi] = trajToTrajMsg(agents[qi]->getTraj(), qi, param.dt);
        }
        pub_desired_trajs_raw.publish(msg_desired_trajs_raw);

        // Vis
        visualization_msgs::MarkerArray msg_desired_trajs_vis;
        for(int qi = 0; qi < mission.qn; qi++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = param.world_frame_id;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = std::to_string(qi);
            marker.id = qi;
            marker.scale.x = 0.05;
            marker.color = mission.color[qi];
            marker.color.a = 0.5;
            marker.pose.orientation = defaultQuaternion();

            double dt = 0.1;
            int n_interval = floor((param.M * param.dt + SP_EPSILON) / dt);
            for (int i = 0; i < n_interval; i++) {
                double future_time = i * dt;
                dynamic_msgs::State traj_point = agents[qi]->getFutureStateMsg(future_time);
                marker.points.emplace_back(traj_point.pose.position);
            }
            msg_desired_trajs_vis.markers.emplace_back(marker);

            //Last point
            marker.points.clear();
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.ns = "last_point";
            marker.id = qi;
            marker.color.a = 0.3;
            marker.scale.x = 2 * mission.agents[qi].radius;
            marker.scale.y = 2 * mission.agents[qi].radius;
            marker.scale.z = 2 * mission.agents[qi].radius * mission.agents[qi].downwash;
            dynamic_msgs::State traj_point = agents[qi]->getFutureStateMsg(param.M * param.dt);
            marker.pose.position = traj_point.pose.position;
            marker.pose.orientation = defaultQuaternion();
            msg_desired_trajs_vis.markers.emplace_back(marker);
        }

        pub_desired_trajs_vis.publish(msg_desired_trajs_vis);
    }

    void MultiSyncSimulator::publishGridMap() {
        if(agents[0]->getPlannerSeq() > 2){
            return;
        }

        if(not param.world_use_octomap) {
            return;
        }

        GridBasedPlanner grid_based_planner(distmap_ptr, mission, param);
        grid_based_planner.plan(mission.agents[0].start_position, mission.agents[0].start_position, 0,
                                mission.agents[0].radius, mission.agents[0].downwash);
        points_t free_grid_points = grid_based_planner.getFreeGridPoints();

        visualization_msgs::MarkerArray msg_grid_map;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        size_t marker_id = 0;
        for(const auto& point : free_grid_points){
            marker.id = marker_id++;

            marker.pose.position = point3DToPointMsg(point);
            marker.pose.orientation = defaultQuaternion();

            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;

            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.color.a = 0.5;

            msg_grid_map.markers.emplace_back(marker);
        }
        pub_grid_map.publish(msg_grid_map);
    }
}