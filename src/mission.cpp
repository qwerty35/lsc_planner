#include <mission.hpp>

namespace DynamicPlanning {
    Mission::Mission(){
        qn = 0;
        on = 0;
    }

    Document Mission::readMissionFile(const std::string& file_name) {
        std::ifstream ifs(file_name);
        IStreamWrapper isw(ifs);
        Document document;
        if (document.ParseStream(isw).HasParseError()) {
            throw std::invalid_argument("There is no such mission file " + file_name + "\n");
        }

        return document;
    }

    bool Mission::initialize(const std::string& mission_file_name_, double max_noise, int world_dimension,
                             double world_z_2d, const std::string& world_file_name_) {
        mission_file_name = mission_file_name_;
        world_file_name = world_file_name_;
        Document document = readMissionFile(mission_file_name_);

        // World
        const Value &world_list = document["world"];
        if(world_list.Size() != 1){
            throw std::invalid_argument("[Mission] World must have one element");
        }
        const Value &dimension = world_list[0].GetObject()["dimension"];
        world_min = octomap::point3d(dimension[0].GetDouble(), dimension[1].GetDouble(), dimension[2].GetDouble());
        world_max = octomap::point3d(dimension[3].GetDouble(), dimension[4].GetDouble(), dimension[5].GetDouble());

        // Quadrotors
        const Value& quadrotor_list = document["quadrotors"];
        for(Value::ConstMemberIterator itr = quadrotor_list.MemberBegin(); itr != quadrotor_list.MemberEnd(); ++itr){
            std::string quad_name = itr->name.GetString();
            Agent quadrotor;

            quadrotor.max_vel.resize(3);
            const Value &maxVel = itr->value.GetObject()["max_vel"];
            for (SizeType i = 0; i < 3; i++) {
                quadrotor.max_vel[i] = maxVel[i].GetDouble();
            }

            quadrotor.max_acc.resize(3);
            const Value &maxAcc = itr->value.GetObject()["max_acc"];
            for (SizeType i = 0; i < 3; i++) {
                quadrotor.max_acc[i] = maxAcc[i].GetDouble();
            }

            quadrotor.radius = itr->value.GetObject()["radius"].GetDouble();
            quadrotor.downwash = itr->value.GetObject()["downwash"].GetDouble();
            quadrotor.nominal_velocity = itr->value.GetObject()["nominal_velocity"].GetDouble();

            quadrotor_map.insert({quad_name, quadrotor});
        }

        // Agents
        const Value &agents_list = document["agents"];
        qn = agents_list.Size();
        agents.resize(qn);
        for (SizeType qi = 0; qi < qn; qi++) {
            // type
            if(agents_list[qi].GetObject()["type"].Empty()){
                ROS_ERROR("[Mission] Agent must have type element");
                return false;
            }
            std::string type = agents_list[qi].GetObject()["type"].GetString();
            agents[qi] = quadrotor_map[type];

            // id
            agents[qi].id = qi;

            // crazyflie id
            if(agents_list[qi].GetObject()["cid"].Empty()){
                agents[qi].cid = qi;
            } else {
                agents[qi].cid = agents_list[qi].GetObject()["cid"].GetInt();
            }

            // start
            if(agents_list[qi].GetObject()["start"].Empty()){
                ROS_ERROR("[Mission] Agent must have start element");
                return false;
            }
            const Value &start = agents_list[qi].GetObject()["start"];
            if(world_dimension == 2){
                agents[qi].start_position = octomap::point3d(start[0].GetDouble(),
                                                             start[1].GetDouble(),
                                                             world_z_2d);
            }
            else{
                agents[qi].start_position = octomap::point3d(start[0].GetDouble(),
                                                             start[1].GetDouble(),
                                                             start[2].GetDouble());
            }


            // goal
            if(agents_list[qi].GetObject()["goal"].Empty()){
                ROS_ERROR("[Mission] Agent must have goal element");
                return false;
            }
            const Value &goal = agents_list[qi].GetObject()["goal"];
            if(world_dimension == 2){
                agents[qi].desired_goal_position = octomap::point3d(goal[0].GetDouble(),
                                                                    goal[1].GetDouble(),
                                                                    world_z_2d);
            }
            else{
                agents[qi].desired_goal_position = octomap::point3d(goal[0].GetDouble(),
                                                                    goal[1].GetDouble(),
                                                                    goal[2].GetDouble());
            }

            // radius
            if(not agents_list[qi].GetObject()["size"].Empty()){
                agents[qi].radius = agents_list[qi].GetObject()["radius"].GetDouble();
            }

            //downwash
            if(not agents_list[qi].GetObject()["downwash"].Empty()){
                agents[qi].downwash = agents_list[qi].GetObject()["downwash"].GetDouble();
            }

            // speed
            if(not agents_list[qi].GetObject()["nominal_velocity"].Empty()){
                agents[qi].nominal_velocity = agents_list[qi].GetObject()["nominal_velocity"].GetDouble();
            }
        }

        initializeAgentColor();

        const Value &obstacle_list = document["obstacles"];
        on = obstacle_list.Size();
        obstacles.resize(on);
        for (SizeType oi = 0; oi < on; oi++) {
            std::string type = obstacle_list[oi].GetObject()["type"].GetString();

            if (type == "spin") {
                geometry_msgs::PoseStamped obs_axis;
                const Value &axis_position = obstacle_list[oi].GetObject()["axis_position"];
                obs_axis.pose.position.x = axis_position[0].GetDouble();
                obs_axis.pose.position.y = axis_position[1].GetDouble();
                obs_axis.pose.position.z = axis_position[2].GetDouble();

                const Value &axis_ori = obstacle_list[oi].GetObject()["axis_ori"];
                obs_axis.pose.orientation.x = axis_ori[0].GetDouble();
                obs_axis.pose.orientation.y = axis_ori[1].GetDouble();
                obs_axis.pose.orientation.z = axis_ori[2].GetDouble();

                geometry_msgs::Point obs_start;
                const Value &start = obstacle_list[oi].GetObject()["start"];
                obs_start.x = start[0].GetDouble();
                obs_start.y = start[1].GetDouble();
                obs_start.z = start[2].GetDouble();

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_speed = obstacle_list[oi].GetObject()["speed"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<SpinObstacle>(obs_axis, obs_start, obs_size, obs_speed, obs_max_acc,
                                                               obs_downwash);
            } else if (type == "straight") {
                geometry_msgs::Point obs_start;
                const Value &start = obstacle_list[oi].GetObject()["start"];
                obs_start.x = start[0].GetDouble();
                obs_start.y = start[1].GetDouble();
                obs_start.z = start[2].GetDouble();

                geometry_msgs::Point obs_goal;
                const Value &goal = obstacle_list[oi].GetObject()["goal"];
                obs_goal.x = goal[0].GetDouble();
                obs_goal.y = goal[1].GetDouble();
                obs_goal.z = goal[2].GetDouble();

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_speed = obstacle_list[oi].GetObject()["speed"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<StraightObstacle>(obs_start, obs_goal, obs_size, obs_speed,
                                                                   obs_max_acc, obs_downwash);
            } else if (type == "multisim_patrol") {
                std::vector<geometry_msgs::Point> obs_waypoints;
                const Value &waypoints = obstacle_list[oi].GetObject()["waypoints"];
                int waypoints_length = waypoints.Size();
                obs_waypoints.resize(waypoints_length);
                for (int i_waypoint = 0; i_waypoint < waypoints_length; i_waypoint++) {
                    const Value &waypoint = waypoints[i_waypoint].GetObject()["waypoint"];
                    geometry_msgs::Point obs_waypoint;
                    obs_waypoint.x = waypoint[0].GetDouble();
                    obs_waypoint.y = waypoint[1].GetDouble();
                    obs_waypoint.z = waypoint[2].GetDouble();
                    obs_waypoints[i_waypoint] = obs_waypoint;
                }

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_speed = obstacle_list[oi].GetObject()["speed"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<PatrolObstacle>(obs_waypoints, obs_size, obs_speed, obs_max_acc,
                                                                 obs_downwash);
            } else if (type == "chasing") {
                dynamic_msgs::Obstacle obs_start_state;
                const Value &start = obstacle_list[oi].GetObject()["start"];
                obs_start_state.pose.position.x = start[0].GetDouble();
                obs_start_state.pose.position.y = start[1].GetDouble();
                obs_start_state.pose.position.z = start[2].GetDouble();

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_max_vel = obstacle_list[oi].GetObject()["max_vel"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_gamma_target = obstacle_list[oi].GetObject()["gamma_target"].GetDouble();
                double obs_gamma_obs = obstacle_list[oi].GetObject()["gamma_obs"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<ChasingObstacle>(obs_start_state, obs_size, obs_max_vel, obs_max_acc,
                                                                  obs_gamma_target, obs_gamma_obs,
                                                                  obs_downwash); //TODO: gamma parameterization
            } else if(type == "gaussian"){
                geometry_msgs::Point obs_start;
                const Value &start = obstacle_list[oi].GetObject()["start"];
                obs_start.x = start[0].GetDouble();
                obs_start.y = start[1].GetDouble();
                obs_start.z = start[2].GetDouble();

                geometry_msgs::Vector3 obs_initial_vel;
                const Value &initial_vel = obstacle_list[oi].GetObject()["initial_vel"];
                obs_initial_vel.x = initial_vel[0].GetDouble();
                obs_initial_vel.y = initial_vel[1].GetDouble();
                obs_initial_vel.z = initial_vel[2].GetDouble();

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_max_vel = obstacle_list[oi].GetObject()["max_vel"].GetDouble();
                double obs_stddev_acc = obstacle_list[oi].GetObject()["stddev_acc"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_acc_update_cycle = obstacle_list[oi].GetObject()["acc_update_cycle"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if(obs_acc_update_cycle == 0){
                    obs_acc_update_cycle = 0.1;
                }
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }
                obstacles[oi] = std::make_shared<GaussianObstacle>(obs_start, obs_size, obs_initial_vel, obs_max_vel,
                                                                   obs_stddev_acc, obs_max_acc, obs_acc_update_cycle,
                                                                   obs_downwash); //TODO: gamma parameterization
            } else if(type == "static"){
                geometry_msgs::Point obs_pose;
                const Value &pose = obstacle_list[oi].GetObject()["pose"];
                obs_pose.x = pose[0].GetDouble();
                obs_pose.y = pose[1].GetDouble();
                obs_pose.z = pose[2].GetDouble();

                geometry_msgs::Point obs_dimensions;
                const Value &dimensions = obstacle_list[oi].GetObject()["dimensions"];
                obs_dimensions.x = dimensions[0].GetDouble();
                obs_dimensions.y = dimensions[1].GetDouble();
                obs_dimensions.z = dimensions[2].GetDouble();

                obstacles[oi] = std::make_shared<StaticObstacle>(obs_pose, obs_dimensions);
            }
            else if(type == "bernstein"){
                std::string traj_csv_path = obstacle_list[oi].GetObject()["traj_csv_path"].GetString();
                int obs_traj_n = obstacle_list[oi].GetObject()["n"].GetInt();
                int obs_cf_id = obstacle_list[oi].GetObject()["cf_id"].GetInt();
                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();

                std::vector<std::vector<octomap::point3d>> total_control_points;
                std::vector<double> time_segments;
                time_segments.emplace_back(0.0);

                std::ifstream traj_csv(traj_csv_path);
                int dim = 2;
                for(auto& row: CSVRange(traj_csv))
                {
                    if(row.size() < 2){
                        break;
                    }
                    std::vector<octomap::point3d> segment_control_points;
                    segment_control_points.resize(obs_traj_n + 1);
                    for(int k = 0; k < dim; k++){
                        for(int i = 0; i < obs_traj_n + 1; i++){
                            std::string token = std::string(row[2 + (obs_traj_n + 1) * k + i]);
                            segment_control_points[i](k) = std::stod(token);
                        }
                    }
                    total_control_points.emplace_back(segment_control_points);
                    time_segments.emplace_back(std::stod(row[1].data()));
                }
                obstacles[oi] = std::make_shared<BernsteinObstacle>(total_control_points, time_segments, obs_size, obs_max_acc, obs_downwash);
            }
            else {
                return false;
            }
        }

        addNoise(max_noise, world_dimension);
        return true;
    }

    void Mission::generateCircleSwap(double circle_radius, int multisim_qn, double z_2d) {
        agents.clear();
        Agent default_agent = defaultAgent();
        for (int qi = 0; qi < multisim_qn; qi++) {
            Agent agent = default_agent;
            agent.id = qi;
            octomap::point3d start_point(circle_radius * std::cos(qi * 2 * PI / multisim_qn),
                                         circle_radius * std::sin(qi * 2 * PI / multisim_qn),
                                         z_2d);
            agent.start_position = start_point;
            agent.desired_goal_position = -start_point;
            agent.desired_goal_position.z() = z_2d;
            addAgent(agent);
        }
    }

    Agent Mission::defaultAgent() const{
        Agent default_agent;
        Document document = readMissionFile(mission_file_name);
        Value::MemberIterator quadrotor = document["quadrotors"].FindMember("default");

        // radius
        default_agent.radius = quadrotor->value.GetObject()["size"].GetDouble();

        //downwash
        default_agent.downwash = quadrotor->value.GetObject()["downwash"].GetDouble();
        if (default_agent.downwash == 0) {
            default_agent.downwash = 1.0;
        }

        // speed
        default_agent.nominal_velocity = quadrotor->value.GetObject()["speed"].GetDouble();
        if (default_agent.nominal_velocity == 0) {
            default_agent.nominal_velocity = 1.0;
        }

        // maximum velocity, acceleration
        std::vector<double> dynamic_limit(3, 0);
        const Value &maxVel = quadrotor->value.GetObject()["max_vel"];
        for (SizeType i = 0; i < maxVel.Size(); i++) {
            dynamic_limit[i] = maxVel[i].GetDouble();
        }
        default_agent.max_vel = dynamic_limit;

        dynamic_limit.assign(3, 0);
        const Value &maxAcc = quadrotor->value.GetObject()["max_acc"];
        for (SizeType i = 0; i < maxAcc.Size(); i++) {
            dynamic_limit[i] = maxAcc[i].GetDouble();
        }
        default_agent.max_acc = dynamic_limit;

        return default_agent;
    }

    void Mission::addAgent(const Agent &agent) {
        agents.emplace_back(agent);
        qn++;
        initializeAgentColor();
    }

    void Mission::addObstacle(const std::shared_ptr<ObstacleBase> &obstacle_ptr) {
        obstacles.emplace_back(obstacle_ptr);
        on++;
    }

    void Mission::addNoise(double max_noise, int dimension) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0, 1);
        for (int qi = 0; qi < qn; qi++) {
            for (int k = 0; k < dimension; k++) {
                agents[qi].desired_goal_position(k) += dis(gen) * max_noise;
            }
        }
    }

    // set color using HSV Colormap
    void Mission::initializeAgentColor() {
        color = getHSVColorMap(qn);
    }

    void Mission::saveMission(const std::string& file_addr) const {
        Document document = readMissionFile(mission_file_name);

        Value& agents_list = document["agents"];
        Document::AllocatorType& allocator = document.GetAllocator();
        if(agents_list.Empty()){
            for(SizeType qi = 0; qi < qn; qi++){
                Value agent(kObjectType);
                Value start(kArrayType);
                Value goal(kArrayType);
                agent.AddMember("type", "crazyflie", allocator);
                for(int i = 0; i < 3; i++){
                    start.PushBack(Value().SetDouble(agents[qi].start_position(i)), allocator);
                    goal.PushBack(Value().SetDouble(agents[qi].desired_goal_position(i)), allocator);
                }
                agent.AddMember("start", start, allocator);
                agent.AddMember("goal", goal, allocator);

                agents_list.PushBack(agent, allocator);
            }
        }
        else{
            for (SizeType qi = 0; qi < qn; qi++) {
                Value& start = agents_list[qi].GetObject()["start"];
                Value& goal = agents_list[qi].GetObject()["goal"];
                for(int i = 0; i < 3; i++){
                    start[i].SetDouble(agents[qi].start_position(i));
                    goal[i].SetDouble(agents[qi].desired_goal_position(i));
                }
            }
        }


        FILE* fp = fopen(file_addr.c_str(), "w");

        char writeBuffer[65536];
        FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

        Writer<FileWriteStream> writer(os);
        document.Accept(writer);

        fclose(fp);
    }
}