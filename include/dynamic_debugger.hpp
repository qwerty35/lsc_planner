#pragma once
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dynamic_msgs/TrajectoryArray.h>
#include <dynamic_msgs/CollisionConstraint.h>
#include <dynamic_msgs/GoalArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <sp_const.hpp>
#include <util.hpp>
#include <collision_constraints.hpp>

using namespace DynamicPlanning;

class DynamicDebugger{
public:
    DynamicDebugger(ros::NodeHandle nh){
        nh.param<std::string>("rosbag_path", rosbag_path, "rosbag.bag");
        nh.param<std::string>("frame_id", frame_id, "world");
        nh.param<int>("agent_id", agent_id, 0);

        std::string prefix = "/mav" + std::to_string(agent_id);
        pub_desired_traj = nh.advertise<visualization_msgs::MarkerArray>("/desired_traj", 1);
        pub_goal = nh.advertise<visualization_msgs::MarkerArray>("/goal_position", 1);
        pub_collision_constraints = nh.advertise<visualization_msgs::MarkerArray>(prefix + "_collision_constraints", 1);
        pub_initial_traj = nh.advertise<visualization_msgs::MarkerArray>(prefix + "_initial_traj", 1);
        pub_obs_pred_traj = nh.advertise<visualization_msgs::MarkerArray>(prefix + "_obs_pred_traj", 1);
        pub_time = nh.advertise<visualization_msgs::Marker>("/time", 1);

        traj_horizon = SP_INFINITY;
    }

    void initialize(){
        agent_radius = 0.15; //TODO: agent_radius
        downwash = 2.0; //TODO: downwash
        current_wall_time = 0;
        reset();
    }

    void reset(){
        traj_start_time = ros::Time::now();
        traj_ready = false;
        traj_finished = false;
    }

    void doStep(){
        current_wall_time += 0.5; //TODO: dt?
    }

    bool isFinished(){
        return traj_finished;
    }

    void publishLSCConstruction(int planner_seq){
        double current_time = (ros::Time::now() - traj_start_time).toSec();
        if(current_time > traj_horizon){
            traj_finished = true;
        }

        if(not traj_ready){
            parseMsg(planner_seq);
            setColorMap(N_obs + 1);
        }
//        publishTime(planner_seq, current_time, "LSC Construction");
        publishTime(planner_seq, current_time, "BVC Construction");
        publishGoal();
        deleteDesiredTraj();
        publishInitialTraj(current_time);
        publishObsPredTraj(current_time);
        publishCollisionConstraints(current_time);
    }

    void publishTrajOptimization(int planner_seq){
        double current_time = (ros::Time::now() - traj_start_time).toSec();
        if(current_time > traj_horizon){
            traj_finished = true;
        }

        if(not traj_ready){
            parseMsg(planner_seq);
            setColorMap(N_obs + 1);
        }
        deleteInitialTraj();
        deleteObsPredTraj();
        publishTime(planner_seq, current_time, "");
        publishDesiredTraj(current_time);
        publishGoal();
        publishCollisionConstraints(current_time);
    }



private:
    ros::Publisher pub_collision_constraints;
    ros::Publisher pub_desired_traj;
    ros::Publisher pub_goal;
    ros::Publisher pub_initial_traj;
    ros::Publisher pub_obs_pred_traj;
    ros::Publisher pub_time;

    std::string rosbag_path, frame_id;
    int agent_id, N_obs;
    double agent_radius, downwash, traj_horizon, current_wall_time;
    dynamic_msgs::TrajectoryArray msg_desired_traj;
    dynamic_msgs::GoalArray msg_goals;
    dynamic_msgs::CollisionConstraint msg_collision_constraints;
    dynamic_msgs::TrajectoryArray msg_obs_pred_traj;
    dynamic_msgs::TrajectoryArray msg_initial_traj;
    std::vector<std_msgs::ColorRGBA> colormap;
    ros::Time traj_start_time, sim_start_time;
    bool traj_ready, traj_finished;

    void parseMsg(int planner_seq){
        rosbag::Bag bag;
        try{
            bag.open(rosbag_path);
        }
        catch(std::exception e){
            ROS_ERROR("[DynamicDebugger] There is no rosbag file");
            return;
        }

        bool traj_updated = false;
        std::vector<std::string> topics;
        std::string prefix = "/mav" + std::to_string(agent_id);
        std::string topic_desired_traj = "/desired_trajs_raw";
        std::string topic_goal_positions = "/goal_positions_raw";
        std::string topic_collision_constraints = prefix + "_collision_constraints_raw";
        std::string topic_obstacle_prediction_traj = prefix + "_obs_pred_traj_raw";
        std::string topic_initial_traj = prefix + "_initial_traj_raw";
        topics.push_back(topic_desired_traj);
        topics.push_back(topic_goal_positions);
        topics.push_back(topic_collision_constraints);
        topics.push_back(topic_obstacle_prediction_traj);
        topics.push_back(topic_initial_traj);
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        for(const rosbag::MessageInstance& m: rosbag::View(bag))
        {
            if(m.getTopic() == topic_desired_traj){
                dynamic_msgs::TrajectoryArray::ConstPtr desired_traj_ptr =
                        m.instantiate<dynamic_msgs::TrajectoryArray>();
                if (desired_traj_ptr != nullptr and desired_traj_ptr->planner_seq == planner_seq){
                    msg_desired_traj = *desired_traj_ptr;
                    traj_horizon = msg_desired_traj.trajectories[0].param.M * msg_desired_traj.trajectories[0].param.dt;
                    traj_updated = true;
                }
            }
            if(m.getTopic() == topic_goal_positions){
                dynamic_msgs::GoalArray::ConstPtr goal_positions_ptr = m.instantiate<dynamic_msgs::GoalArray>();
                if (goal_positions_ptr != nullptr and goal_positions_ptr->goals[0].planner_seq == planner_seq){
                    msg_goals = *goal_positions_ptr;
                    traj_updated = true;
                }
            }
            if(m.getTopic() == topic_collision_constraints){
                dynamic_msgs::CollisionConstraint::ConstPtr collision_constraints_ptr =
                        m.instantiate<dynamic_msgs::CollisionConstraint>();
                if (collision_constraints_ptr != nullptr and collision_constraints_ptr->planner_seq == planner_seq) {
                    msg_collision_constraints = *collision_constraints_ptr;
                    traj_updated = true;
                }
            }
            if(m.getTopic() == topic_obstacle_prediction_traj){
                dynamic_msgs::TrajectoryArray::ConstPtr obs_pred_traj_ptr =
                        m.instantiate<dynamic_msgs::TrajectoryArray>();
                if (obs_pred_traj_ptr != nullptr and obs_pred_traj_ptr->planner_seq == planner_seq) {
                    msg_obs_pred_traj = *obs_pred_traj_ptr;
                    N_obs = msg_obs_pred_traj.trajectories.size();
                    traj_updated = true;
                }
            }
            if(m.getTopic() == topic_initial_traj){
                dynamic_msgs::TrajectoryArray::ConstPtr initial_traj_ptr =
                        m.instantiate<dynamic_msgs::TrajectoryArray>();
                if (initial_traj_ptr != nullptr and initial_traj_ptr->planner_seq == planner_seq) {
                    msg_initial_traj = *initial_traj_ptr;
                    traj_updated = true;
                }
            }
        }
        bag.close();

        if(traj_updated){
            traj_ready = true;
        }
    }

    void setColorMap(int size){
        colormap = getHSVColorMap(size);
    }

    void publishTime(int planner_seq, double current_time, std::string state){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 1;
        marker.scale.z = 0.5;
//        marker.text = state + "\nplanner_iter: " + std::to_string(planner_seq) + "\nplanner_time: " + std::to_string(current_time);
        marker.text = "Iteration: " + std::to_string(planner_seq) + "          \n" + state;
        marker.pose.position.x = -3.0;
        marker.pose.position.y = -1.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = defaultQuaternion();
        pub_time.publish(marker);
    }

    void publishDesiredTraj(double current_time){
        visualization_msgs::MarkerArray msg_desired_traj_vis =
                convertTrajToMarkerArray(msg_desired_traj, current_time, 1.0);
        pub_desired_traj.publish(msg_desired_traj_vis);
    }

    visualization_msgs::MarkerArray convertTrajToMarkerArray(const dynamic_msgs::TrajectoryArray& msg_traj_array,
                                                             double a, bool use_colormap = true){
        visualization_msgs::MarkerArray msg_traj_vis;
        if(msg_traj_array.trajectories.empty()){
            return msg_traj_vis;
        }

        for(const auto& msg_traj : msg_traj_array.trajectories){
            int id = msg_traj.param.id;
            double dt = msg_traj.param.dt;
            traj_t traj = trajMsgToTraj(msg_traj);
            visualization_msgs::Marker marker_traj;
            std_msgs::ColorRGBA color;
            if(use_colormap){
                color = colormap[id];
            }
            else{
                color.r = 0;
                color.g = 0;
                color.b = 0;
            }
            color.a = a;
            marker_traj = trajToMarkerMsg(traj, dt, frame_id, id, color);
            msg_traj_vis.markers.emplace_back(marker_traj);
        }

        return msg_traj_vis;
    }

    void publishGoal(){
        visualization_msgs::MarkerArray msg_goal_vis;
        for(const auto& msg_goal: msg_goals.goals){
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "current_goal";
            marker.id = msg_goal.id;
            marker.color = colormap[msg_goal.id];
            marker.color.a = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.pose.position = msg_goal.current_goal;
            marker.pose.orientation = defaultQuaternion();
            msg_goal_vis.markers.emplace_back(marker);

            marker.type = visualization_msgs::Marker::CUBE;
            marker.ns = "desired_goal";
            marker.pose.position = msg_goal.desired_goal;
            msg_goal_vis.markers.emplace_back(marker);
        }

        pub_goal.publish(msg_goal_vis);
    }

    visualization_msgs::MarkerArray convertTrajToMarkerArray(const dynamic_msgs::TrajectoryArray& msg_traj_array,
                                                             double current_time, double a, bool use_colormap = true){
        visualization_msgs::MarkerArray msg_traj_vis;
        if(msg_traj_array.trajectories.empty()){
            return msg_traj_vis;
        }

        for(const auto& msg_traj : msg_traj_array.trajectories){
            int id = msg_traj.param.id;
            int M = msg_traj.param.M;
            int n = msg_traj.param.n;
            double dt = msg_traj.param.dt;

            // traj
            traj_t traj = trajMsgToTraj(msg_traj);
            std_msgs::ColorRGBA color;
            if(use_colormap){
                color = colormap[id];
            }
            else{
                color.r = 0;
                color.g = 0;
                color.b = 0;
            }
            color.a = a;
            visualization_msgs::Marker marker_traj = trajToMarkerMsg(traj, dt, frame_id, id, color, current_time);
            msg_traj_vis.markers.emplace_back(marker_traj);

            // collision model
            double t = std::min(current_time, M * dt);
            dynamic_msgs::State state = getStateFromControlPoints(traj, t, M, n, dt);
            point_t position = pointMsgToPoint3d(state.pose.position);
            msg_traj_vis.markers.emplace_back(convertCollisionModelToMarker(position, agent_radius, id, use_colormap));
        }

        return msg_traj_vis;
    }

    void publishCollisionConstraints(double current_time){
        visualization_msgs::MarkerArray msg1 = convertRSFCToMarkerArray(current_time);
//        visualization_msgs::MarkerArray msg2 = convertSFCToMarkerArray(current_time);
//        msg1.markers.insert(msg1.markers.end(), msg2.markers.begin(), msg2.markers.end());
        pub_collision_constraints.publish(msg1);
    }

    visualization_msgs::MarkerArray convertRSFCToMarkerArray(double current_time){
        visualization_msgs::MarkerArray msg_rsfc_vis;

        if(msg_collision_constraints.rsfcs.empty()){
            return msg_rsfc_vis;
        }

        // update parameters
        int M = msg_collision_constraints.rsfcs[0].param.M;
        int n = msg_collision_constraints.rsfcs[0].param.n;
        double dt = msg_collision_constraints.rsfcs[0].param.dt;
        int m = static_cast<int>(current_time / dt);
        double t_normalized = (current_time - m * dt) / dt;
        if (m > M - 1) {
            m = M - 1;
            t_normalized = 1;
        }

        msg_rsfc_vis.markers.clear();
        for (int oi = 0; oi < N_obs; oi++) {
            // Parsing message
            //obs_id
            int obs_id = msg_collision_constraints.rsfcs[oi].param.id;

            // obstacle position
            points_t obs_control_points;
            obs_control_points.resize(n + 1);
            for (int i = 0; i < n + 1; i++) {
                obs_control_points[i] = pointMsgToPoint3d(msg_collision_constraints.rsfcs[oi].obs_control_points[m * (n + 1) + i]);
            }
            point_t obs_position = getPointFromControlPoints(obs_control_points, t_normalized);

            // normal vector
            // use minus normal vector for showing collision region
            point_t normal_vector = -pointMsgToPoint3d(msg_collision_constraints.rsfcs[oi].normal_vector[m * (n + 1)]);

            // safe distance
            std::vector<double> d;
            d.resize(n + 1);
            for (int i = 0; i < n + 1; i++) {
                d[i] = msg_collision_constraints.rsfcs[oi].d[m * (n + 1) + i];
            }
            double obs_size = getPointFromControlPoints(d, t_normalized) - agent_radius;

            // Generate Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color = colormap[obs_id];
            marker.color.a = 0.2;
            double box_scale = 40;
            marker.scale.x = box_scale;
            marker.scale.y = box_scale;
            marker.scale.z = box_scale;

            double distance = -obs_size + box_scale / 2;
            Eigen::Vector3d V3d_normal_vector(normal_vector.x(), normal_vector.y(), normal_vector.z());
            Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, V3d_normal_vector);

            marker.id = oi;
            marker.ns = "RSFC";
            marker.pose.position = point3DToPointMsg(obs_position + normal_vector.normalized() * distance);
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();
            msg_rsfc_vis.markers.emplace_back(marker);

            // Obstacle collision model
            msg_rsfc_vis.markers.emplace_back(convertCollisionModelToMarker(obs_position, agent_radius, obs_id));
        }

        return msg_rsfc_vis;
    }

    visualization_msgs::Marker convertCollisionModelToMarker(point_t position, double size, int id, bool use_colormap = true){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        std_msgs::ColorRGBA color;
        if(use_colormap){
            color = colormap[id];
        }
        else{
            color.r = 0;
            color.g = 0;
            color.b = 0;
        }
        color.a = 0.2;
        marker.color = color;

        marker.id = id;
        marker.ns = "collision_model";

        marker.scale.x = 2 * size;
        marker.scale.y = 2 * size;
        marker.scale.z = 2 * size * downwash;
        marker.pose.position = point3DToPointMsg(position);
        marker.pose.orientation = defaultQuaternion();
        return marker;
    }

    visualization_msgs::MarkerArray convertSFCToMarkerArray(double current_time){
        visualization_msgs::MarkerArray msg_sfc_vis;

        int M = msg_collision_constraints.sfcs[0].param.M;
        double dt = msg_collision_constraints.sfcs[0].param.dt;
        int m = static_cast<int>(current_time / dt);
        if (m > M - 1) {
            m = M - 1;
        }

        //Parsing message
        point_t box_min = pointMsgToPoint3d(msg_collision_constraints.sfcs[m].box_min);
        point_t box_max = pointMsgToPoint3d(msg_collision_constraints.sfcs[m].box_max);
        SFC box(box_min, box_max);

        visualization_msgs::Marker marker = box.convertToMarker(agent_radius, "world");
        marker.id = 0;
        marker.ns = "SFC";
        marker.color = colormap[agent_id];
        marker.color.a = 1.0;
        msg_sfc_vis.markers.emplace_back(marker);
        return msg_sfc_vis;
    }

    void publishInitialTraj(double current_time){
        visualization_msgs::MarkerArray msg_intial_traj_vis = convertTrajToMarkerArray(msg_initial_traj, current_time, 0.5, false);
        pub_initial_traj.publish(msg_intial_traj_vis);
    }

    void publishObsPredTraj(double current_time){
        visualization_msgs::MarkerArray msg_obs_pred_traj_vis = convertTrajToMarkerArray(msg_obs_pred_traj, current_time, 0.5, false);
        pub_obs_pred_traj.publish(msg_obs_pred_traj_vis);
    }

    void deleteInitialTraj(){
        visualization_msgs::MarkerArray msg_intial_traj_vis;
//        for(const auto& msg_traj : msg_initial_traj.trajectories){
//            visualization_msgs::Marker marker;
//            marker.action = visualization_msgs::Marker::DELETE;
//            marker.id = msg_traj.param.id;
//            marker.ns = "collision_model";
//            msg_intial_traj_vis.markers.emplace_back(marker);
//        }

        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        msg_intial_traj_vis.markers.emplace_back(marker);
        pub_initial_traj.publish(msg_intial_traj_vis);
    }

    void deleteObsPredTraj(){
        visualization_msgs::MarkerArray msg_obs_pred_traj_vis;
//        for(const auto& msg_traj : msg_obs_pred_traj.trajectories){
//            visualization_msgs::Marker marker;
//            marker.action = visualization_msgs::Marker::DELETE;
//            marker.id = msg_traj.param.id;
//            marker.ns = "collision_model";
//            msg_obs_pred_traj_vis.markers.emplace_back(marker);
//        }

        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        msg_obs_pred_traj_vis.markers.emplace_back(marker);
        pub_obs_pred_traj.publish(msg_obs_pred_traj_vis);
    }

    void deleteDesiredTraj(){
        visualization_msgs::MarkerArray msg_desired_traj_vis;
//        for(const auto& msg_traj : msg_desired_traj.trajectories){
//            visualization_msgs::Marker marker;
//            marker.action = visualization_msgs::Marker::DELETE;
//            marker.id = msg_traj.param.id;
//            marker.ns = "collision_model";
//            msg_desired_traj_vis.markers.emplace_back(marker);
//        }

        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        msg_desired_traj_vis.markers.emplace_back(marker);
        pub_desired_traj.publish(msg_desired_traj_vis);
    }
};