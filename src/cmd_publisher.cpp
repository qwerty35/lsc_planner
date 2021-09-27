#include "cmd_publisher.hpp"

namespace DynamicPlanning {
    CmdPublisher::CmdPublisher(const ros::NodeHandle &_nh, Mission _mission) : nh(_nh), mission(std::move(_mission))
    {
        sub_desired_trajs = nh.subscribe("/desired_trajs_raw", 1, &CmdPublisher::trajsCallback, this);
        service_stop_planning = nh.advertiseService("/stop_planning", &CmdPublisher::stopPlanningCallback, this);

        pubs_cmd.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++){
            pubs_cmd[qi] = nh.advertise<dynamic_msgs::FullState>("/cf" + std::to_string(mission.agents[qi].cid) + "/cmd_full_state", 1);
        }
        pub_cmd_vis = nh.advertise<visualization_msgs::MarkerArray>("/cmd_full_state_vis", 1);

        latest_planner_seq = 0;
        M = 0;
        n = 0;
        dt = 0;
        stop_planning = false;
    }

    void CmdPublisher::run() {
        update_traj();
        publish_traj();
    }

    void CmdPublisher::trajsCallback(const dynamic_msgs::TrajectoryArray& msg_trajs) {
        if(msg_trajs.planner_seq <= latest_planner_seq) {
            return; // msg not updated
        }

        if(msg_trajs.trajectories.size() != mission.qn) {
            throw std::invalid_argument("[CmdPublisher] Invalid msg, mission qn != msg_desired_traj.size()");
        }

        if(M == 0){ // if msg is first received
            M = msg_trajs.trajectories[0].param.M;
            n = msg_trajs.trajectories[0].param.n;
            dt = msg_trajs.trajectories[0].param.dt;
        }

        latest_planner_seq = msg_trajs.planner_seq;

        std::vector<traj_t> desired_trajs;
        desired_trajs.resize(mission.qn);
        for(int qi = 0; qi < mission.qn; qi++){
            desired_trajs[qi] = trajMsgToTraj(msg_trajs.trajectories[qi]);
        }

        trajs_queue.push(desired_trajs);
        trajs_start_time_queue.push(msg_trajs.header.stamp);
    }

    bool CmdPublisher::stopPlanningCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        stop_planning = true;
        return true;
    }

    void CmdPublisher::update_traj(){
        if(trajs_queue.empty()){
            return;
        }

        if(current_traj.empty() or ros::Time::now() > trajs_start_time_queue.front()){
            current_traj = trajs_queue.front();
            current_traj_start_time = trajs_start_time_queue.front();

            trajs_queue.pop();
            trajs_start_time_queue.pop();
        }
    }

    void CmdPublisher::publish_traj(){
        if(current_traj.empty() or stop_planning){
            return;
        }

        double t = (ros::Time::now() - current_traj_start_time).toSec();
        if(t < 0){
            return;
        }

        visualization_msgs::MarkerArray msg_cmd_vis;
        for(int qi = 0; qi < mission.qn; qi++){
            dynamic_msgs::State desired_state;
            if(t > M * dt){
                desired_state = getStateFromControlPoints(current_traj[qi], M * dt, M, n, dt);
                desired_state.velocity.linear = defaultVector();
                desired_state.velocity.angular = defaultVector();
                desired_state.acceleration.linear = defaultVector();
                desired_state.acceleration.linear = defaultVector();
            }
            else{
                desired_state = getStateFromControlPoints(current_traj[qi], t, M, n, dt);
            }

            // command for crazyflie
            dynamic_msgs::FullState msg_cmd;
            msg_cmd.pose.position = desired_state.pose.position;
            msg_cmd.pose.orientation = defaultQuaternion();
            msg_cmd.twist = desired_state.velocity;
            msg_cmd.acc = desired_state.acceleration.linear;
            pubs_cmd[qi].publish(msg_cmd);

            // visualization
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.ns = std::to_string(qi);
            marker.id = qi;
            marker.color = mission.color[qi];
            marker.color.a = 1.0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.05;
            marker.scale.y = 0.1;
            marker.pose.position = defaultPoint();
            marker.pose.orientation = defaultQuaternion();

            geometry_msgs::Point start_point, end_point;
            start_point = desired_state.pose.position;
            end_point = point3DToPointMsg(pointMsgToPoint3d(desired_state.pose.position) + vector3MsgToPoint3d(desired_state.velocity.linear) * 1.0);
            marker.points.emplace_back(start_point);
            marker.points.emplace_back(end_point);
            msg_cmd_vis.markers.emplace_back(marker);
        }
        pub_cmd_vis.publish(msg_cmd_vis);
    }
}