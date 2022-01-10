#include "cmd_publisher.hpp"

namespace DynamicPlanning {
    CmdPublisher::CmdPublisher(const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission, int _agent_id)
        : nh(_nh), param(_param), mission(_mission), agent_id(_agent_id)
    {
        nh.param<double>("landing_time", landing_time, 3.0);

        cmd_timer = nh.createTimer(ros::Duration(0.02), &CmdPublisher::cmdTimerCallback, this);
        pub_cmd = nh.advertise<dynamic_msgs::FullState>("/cf" + std::to_string(mission.agents[agent_id].cid) + "/cmd_full_state", 1);
        pub_cmd_stop = nh.advertise<std_msgs::Empty>("/cf" + std::to_string(mission.agents[agent_id].cid) + "/cmd_stop", 1);
        pub_cmd_vis = nh.advertise<visualization_msgs::MarkerArray>("/cmd_full_state_vis", 1);

        M = param.M;
        n = param.n;
        dt = param.dt;
        initialized = false;
        external_pose_update = false;
        is_disturbed = false;
        landing = false;
    }

    void CmdPublisher::updateTraj(const traj_t& new_traj, const ros::Time& traj_start_time){
        if(not initialized){
            initialized = true;
        }

        traj_queue.push(new_traj);
        traj_start_time_queue.push(traj_start_time);
    }

    void CmdPublisher::landingCallback(){
        if(not landing){
            landing = true;
            landing_start_time = ros::Time::now();
        }
    }

    bool CmdPublisher::isDisturbed() const{
        return is_disturbed;
    }

    bool CmdPublisher::externalPoseUpdate() const{
        return external_pose_update;
    }

    bool CmdPublisher::landingFinished() const {
        return (ros::Time::now() - landing_start_time).toSec() > landing_time;
    }

    point3d CmdPublisher::getObservedPosition() const {
        if(not external_pose_update){
            throw std::invalid_argument("[CmdPublisher] external pose is not updated");
        }
        return observed_position;
    }

    void CmdPublisher::cmdTimerCallback(const ros::TimerEvent& event) {
        observeCurrentPosition();
        loadCurrentTraj();

        dynamic_msgs::State desired_state;
        bool success = computeDesiredState(desired_state);
        if(success){
            detectDisturbance(desired_state);

            if(landing){
                publishLandingCommand(desired_state);
            } else {
                publishCommand(desired_state);
            }
        }
        else if(initialized and external_pose_update and current_traj.empty()){
            failsafe();
        }
    }

    void CmdPublisher::observeCurrentPosition() {
        // listen tf and get current position
        // if there is no external pose update, then use ideal state of agent instead.
        tf::StampedTransform transform;
        try {
            tf_listener.lookupTransform(param.world_frame_id, "/cf" + std::to_string(mission.agents[agent_id].cid),
                                        ros::Time(0), transform);
            observed_position = vector3MsgToPoint3d(transform.getOrigin());
            external_pose_update = true;
        }
        catch (tf::TransformException &ex) {
            ROS_WARN_ONCE("[MultiSyncSimulator] tf listener failed, use ideal state instead.");
            external_pose_update = false;
        }
    }

    bool CmdPublisher::computeDesiredState(dynamic_msgs::State& desired_state){
        // Check current trajectory is received
        if (current_traj.empty()) {
//            ROS_WARN("[CmdPublisher] There is no current trajectory, start planning first!");
            return false;
        }

        // time
        double t;
        t = (ros::Time::now() - current_traj_start_time).toSec();
        if (t < 0) {
            return false;
        }

        if (t > M * dt) {
            desired_state = getStateFromControlPoints(current_traj, M * dt, M, n, dt);
            desired_state.velocity.linear = defaultVector();
            desired_state.velocity.angular = defaultVector();
            desired_state.acceleration.linear = defaultVector();
        } else {
            desired_state = getStateFromControlPoints(current_traj, t, M, n, dt);
        }

        return true;
    }

    void CmdPublisher::detectDisturbance(dynamic_msgs::State& desired_state){
        if(not external_pose_update or landing){
            is_disturbed = false;
            return;
        }

        // if diff between ideal and current position is under threshold, use ideal state
        // else use current_state
        point3d desired_current_position = pointMsgToPoint3d(desired_state.pose.position);
        double diff = observed_position.distance(desired_current_position);
        if ((not is_disturbed and diff > param.reset_threshold) or (is_disturbed and diff > 0.05)) {
            is_disturbed = true;
        } else {
            is_disturbed = false;
        }

        if(is_disturbed){
            current_traj.clear();
            std::queue<traj_t> empty_traj_queue;
            std::queue<ros::Time> empty_time_queue;
            std::swap(traj_queue, empty_traj_queue);
            std::swap(traj_start_time_queue, empty_time_queue);

            desired_state.pose.position = point3DToPointMsg(observed_position);
            desired_state.pose.orientation = defaultQuaternion();
            desired_state.velocity.linear = defaultVector();
            desired_state.acceleration.linear = defaultVector();
        }
    }


    void CmdPublisher::loadCurrentTraj(){
        if(traj_queue.empty()){
            return;
        }

        if(current_traj.empty() or ros::Time::now() > traj_start_time_queue.front()){
            current_traj = traj_queue.front();
            current_traj_start_time = traj_start_time_queue.front();

            traj_queue.pop();
            traj_start_time_queue.pop();
        }
    }

    void CmdPublisher::publishCommand(const dynamic_msgs::State& desired_state) {
        // command for crazyflie
        dynamic_msgs::FullState msg_cmd;
        msg_cmd.pose.position = desired_state.pose.position;
        msg_cmd.pose.orientation = defaultQuaternion();
        msg_cmd.twist = desired_state.velocity;
        msg_cmd.acc = desired_state.acceleration.linear;
        pub_cmd.publish(msg_cmd);

        // visualization
        visualization_msgs::MarkerArray msg_cmd_vis;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = std::to_string(agent_id);
        marker.id = agent_id;
        marker.color = mission.color[agent_id];
        marker.color.a = 1.0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.1;
        marker.pose.position = defaultPoint();
        marker.pose.orientation = defaultQuaternion();

        geometry_msgs::Point start_point, end_point;
        start_point = desired_state.pose.position;
        end_point = point3DToPointMsg(pointMsgToPoint3d(desired_state.pose.position) +
                                      vector3MsgToPoint3d(desired_state.velocity.linear) * 1.0);
        marker.points.emplace_back(start_point);
        marker.points.emplace_back(end_point);
        msg_cmd_vis.markers.emplace_back(marker);
        pub_cmd_vis.publish(msg_cmd_vis);
    }

    void CmdPublisher::publishLandingCommand(const dynamic_msgs::State& new_state){
        // Don't consider the disturbance while landing
        double t_land;
        t_land = (ros::Time::now() - landing_start_time).toSec();

        if(t_land > landing_time){
            std_msgs::Empty msg_stop;
            pub_cmd_stop.publish(msg_stop);
        }
        else{
            dynamic_msgs::State desired_state = new_state;
            desired_state.pose.position.z =
                    0.03 + (desired_state.pose.position.z - 0.03) * std::max(1 - t_land / landing_time, 0.0);
            desired_state.velocity.linear = defaultVector();
            desired_state.velocity.angular = defaultVector();
            desired_state.acceleration.linear = defaultVector();
            publishCommand(desired_state);
        }
    }

    void CmdPublisher::failsafe(){
        dynamic_msgs::State desired_state;
        desired_state.pose.position = point3DToPointMsg(observed_position);
        desired_state.pose.orientation = defaultQuaternion();
        desired_state.velocity.linear = defaultVector();
        desired_state.acceleration.linear = defaultVector();

        publishCommand(desired_state);
    }
}