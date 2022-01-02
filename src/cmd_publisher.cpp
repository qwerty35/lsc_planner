#include "cmd_publisher.hpp"

namespace DynamicPlanning {
    CmdPublisher::CmdPublisher(const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission, int _agent_id)
        : nh(_nh), param(_param), mission(_mission), agent_id(_agent_id)
    {
        nh.param<double>("landing_time", landing_time, 5.0);

        cmd_timer = nh.createTimer(ros::Duration(0.02), &CmdPublisher::cmdTimerCallback, this);
        pub_cmd = nh.advertise<dynamic_msgs::FullState>("/cf" + std::to_string(mission.agents[agent_id].cid) + "/cmd_full_state", 1);
        pub_cmd_vis = nh.advertise<visualization_msgs::MarkerArray>("/cmd_full_state_vis", 1);

        M = param.M;
        n = param.n;
        dt = param.dt;
        is_disturbed = false;
        stop_planning = false;
    }

    void CmdPublisher::updateTraj(const traj_t& new_traj, const ros::Time& traj_start_time, bool _is_disturbed){
        traj_queue.push(new_traj);
        traj_start_time_queue.push(traj_start_time);
        is_disturbed = _is_disturbed;
    }

    void CmdPublisher::cmdTimerCallback(const ros::TimerEvent& event) {
        loadCurrentTraj();
        publishCommand();
    }

    void CmdPublisher::stopPlanningCallback(){
        stop_planning = true;
        stop_planning_time = ros::Time::now();
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

    void CmdPublisher::publishCommand() {
        // Check current trajectory is received
        if (current_traj.empty()) {
            return;
        }

        // time
        double t, t_stop;
        if (stop_planning) {
            t = (stop_planning_time - current_traj_start_time).toSec();
            t_stop = (ros::Time::now() - stop_planning_time).toSec();
        } else {
            t = (ros::Time::now() - current_traj_start_time).toSec();
            t_stop = 0;
        }
        if (t < 0 or t_stop > landing_time) {
            return;
        }

        dynamic_msgs::State desired_state;
        if (t > M * dt) {
            desired_state = getStateFromControlPoints(current_traj, M * dt, M, n, dt);
            desired_state.velocity.linear = defaultVector();
            desired_state.velocity.angular = defaultVector();
            desired_state.acceleration.linear = defaultVector();
        } else {
            desired_state = getStateFromControlPoints(current_traj, t, M, n, dt);
        }

        if (stop_planning) {
            // Landing
            desired_state.pose.position.z =
                    0.01 + (desired_state.pose.position.z - 0.01) * std::max(1 - t_stop / landing_time, 0.0);
            desired_state.velocity.linear = defaultVector();
            desired_state.velocity.angular = defaultVector();
            desired_state.acceleration.linear = defaultVector();
        }

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
}