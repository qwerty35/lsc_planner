#ifndef LSC_PLANNER_UTIL_HPP
#define LSC_PLANNER_UTIL_HPP

#include <sp_const.hpp>
#include <polynomial.hpp>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <dynamic_msgs/Trajectory.h>
#include <dynamic_msgs/State.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Vector3.h>

namespace DynamicPlanning {
    static geometry_msgs::Point defaultPoint(){
        geometry_msgs::Point msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        return msg;
    }

    static geometry_msgs::Vector3 defaultVector(){
        geometry_msgs::Vector3 msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        return msg;
    }


    static geometry_msgs::Quaternion defaultQuaternion(){
        geometry_msgs::Quaternion msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        msg.w = 1;
        return msg;
    }

    static octomap::point3d pointMsgToPoint3d(const geometry_msgs::Point& msg){
        return octomap::point3d(msg.x, msg.y, msg.z);
    }

    static octomap::point3d vector3MsgToPoint3d(const geometry_msgs::Vector3& msg){
        return octomap::point3d(msg.x, msg.y, msg.z);
    }

    static octomap::point3d vector3MsgToPoint3d(const tf::Vector3& vector){
        return octomap::point3d(vector.x(), vector.y(), vector.z());
    }

    static geometry_msgs::Point vector3MsgToPointMsg(const tf::Vector3& vector){
        geometry_msgs::Point point;
        point.x = vector.x();
        point.y = vector.y();
        point.z = vector.z();
        return point;
    }

    static geometry_msgs::Vector3 point3DToVector3Msg(const octomap::point3d& point){
        geometry_msgs::Vector3 vector;
        vector.x = point.x();
        vector.y = point.y();
        vector.z = point.z();
        return vector;
    }

    static geometry_msgs::Point point3DToPointMsg(const octomap::point3d& point){
        geometry_msgs::Point msg;
        msg.x = point.x();
        msg.y = point.y();
        msg.z = point.z();
        return msg;
    }

    static geometry_msgs::Twist point3DToTwistMsg(const octomap::point3d& point){
        geometry_msgs::Twist msg;
        msg.linear.x = point.x();
        msg.linear.y = point.y();
        msg.linear.z = point.z();
        return msg;
    }

    static geometry_msgs::Quaternion point3DToQuaternionMsg(const octomap::point3d& point){
        geometry_msgs::Quaternion msg;
        msg.x = point.x();
        msg.y = point.y();
        msg.z = point.z();
        msg.w = sqrt(1 - point.norm_sq());
        return msg;
    }

    static octomap::point3d quaternionMsgToPoint3D(const geometry_msgs::Quaternion& msg){
        return octomap::point3d(msg.x, msg.y, msg.z);
    }

    static geometry_msgs::Quaternion quaternionToQuaternionMsg(const Eigen::Quaterniond& q){
        geometry_msgs::Quaternion msg;
        msg.x = q.x();
        msg.y = q.y();
        msg.z = q.z();
        msg.w = q.w();
        return msg;
    }

    static traj_t trajMsgToTraj(const dynamic_msgs::Trajectory& msg){
        size_t M = msg.param.M;
        size_t n = msg.param.n;
        if(msg.control_points.size() != M * (n + 1)){
            throw std::invalid_argument("[trajMsgToTraj] control points is not match to msg's parameters");
        }

        traj_t traj;
        traj.resize(M);
        for(size_t m = 0; m < M; m++){
            traj[m].resize(n + 1);
            for(size_t i = 0; i < n + 1; i++){
                traj[m][i] = pointMsgToPoint3d(msg.control_points[m * (n + 1) + i]);
            }
        }

        return traj;
    }

    static dynamic_msgs::Trajectory trajToTrajMsg(const traj_t& traj, int id, double dt){
        dynamic_msgs::Trajectory msg_traj;

        if(traj.empty()){
            throw std::invalid_argument("[TrajToTrajMsg] trajectory is empty");
        }

        int M = traj.size();
        int n = traj[0].size() - 1;

        msg_traj.param.id = id;
        msg_traj.param.M = M;
        msg_traj.param.n = n;
        msg_traj.param.dt = dt;

        msg_traj.control_points.resize(M * (n + 1));
        for(int m = 0; m < M; m++){
            for(int i = 0; i < n + 1; i++){
                msg_traj.control_points[m * (n + 1) + i] = point3DToPointMsg(traj[m][i]);
            }
        }

        return msg_traj;
    }

    static visualization_msgs::Marker trajToMarkerMsg(const traj_t& traj,
                                                      double dt,
                                                      const std::string& frame_id,
                                                      int agent_id,
                                                      std_msgs::ColorRGBA color,
                                                      double max_horizon = SP_INFINITY){
        if(traj.empty()){
            throw std::invalid_argument("[trajToMarkerArrayMsg] trajectory is empty");
        }

        int M = traj.size();
        int n = traj[0].size() - 1;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = std::to_string(agent_id);
        marker.id = agent_id;
        marker.scale.x = 0.05;
        marker.color = color;
        marker.pose.orientation = defaultQuaternion();

        double dt_interval = 0.05;
        double horizon = std::min(M * dt, max_horizon);
        int n_interval = floor((horizon + SP_EPSILON) / dt_interval);

        dynamic_msgs::State state;
        state = getStateFromControlPoints(traj, 0, M, n, dt);
        marker.points.emplace_back(state.pose.position);
        for (int i = 0; i < n_interval; i++) {
            double future_time = i * dt_interval;
            state = getStateFromControlPoints(traj, future_time, M, n, dt);
            marker.points.emplace_back(state.pose.position);
        }
        state = getStateFromControlPoints(traj, horizon, M, n, dt);
        marker.points.emplace_back(state.pose.position);

        return marker;
    }

    static std::vector<std::array<double, 3>> point3DsToArray(const std::vector<octomap::point3d>& points){
        /* Allocate memory. */
        std::vector<std::array<double, 3>> arrays;

        /* Read and store vertices' coordinates. */
        for (const auto& point : points)
        {
            std::array<double,3> array = {{point.x(), point.y(), point.z()}};
            arrays.emplace_back(array);
        }

        return arrays;
    }

    static dynamic_msgs::State odomMsgToState(const nav_msgs::Odometry& msg){
        dynamic_msgs::State state;
        state.pose = msg.pose.pose;
        state.velocity = msg.twist.twist;
        state.acceleration.linear = defaultVector();
        state.acceleration.angular = defaultVector();

        return state;
    }

    template <typename T>
    bool isElementInVector(std::vector<T> vec, T element){
        return std::find(vec.begin(), vec.end(), element) != vec.end();
    }

    template <typename T>
    bool isElementInSet(std::set<T> set, T element){
        return set.find(element) != set.end();
    }

    static double distBetweenAgents(const octomap::point3d& p1, const octomap::point3d& p2, double downwash){
        octomap::point3d delta = p1 - p2;
        delta.z() = delta.z() / downwash;
        return delta.norm();
    }

    static traj_t coordinateTransform(const traj_t& traj, double downwash){
        traj_t traj_trans = traj;
        for(auto & segment : traj_trans){
            for(auto & control_point : segment){
                control_point.z() = control_point.z() / downwash;
            }
        }

        return traj_trans;
    }

    static std::vector<std_msgs::ColorRGBA> getHSVColorMap(int size){
        std::vector<std_msgs::ColorRGBA> color;
        double h, f;
        int i;

        color.resize(size);
        for (int idx = 0; idx < size; idx++) {
            h = idx * 6 / (double) size;
            i = (int) h;
            f = h - i;

            switch (i) {
                case 0:
                    color[idx].r = 1;
                    color[idx].g = f;
                    color[idx].b = 0;
                    break;
                case 1:
                    color[idx].r = 1 - f;
                    color[idx].g = 1;
                    color[idx].b = 0;
                    break;
                case 2:
                    color[idx].r = 0;
                    color[idx].g = 1;
                    color[idx].b = f;
                    break;
                case 3:
                    color[idx].r = 0;
                    color[idx].g = 1 - f;
                    color[idx].b = 1;
                    break;
                case 4:
                    color[idx].r = f;
                    color[idx].g = 0;
                    color[idx].b = 1;
                    break;
                case 5:
                    color[idx].r = 1;
                    color[idx].g = 0;
                    color[idx].b = 1 - f;
                    break;
                default:
                    break;
            }
        }

        return color;
    }
}

#endif //LSC_PLANNER_UTIL_HPP
