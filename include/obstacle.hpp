#pragma once
#include <dynamic_msgs/Trajectory.h>
#include <dynamic_msgs/Obstacle.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sp_const.hpp>
#include <polynomial.hpp>
#include <utility>
#include <random>

namespace DynamicPlanning {
    class ObstacleBase {
    public:
        //DynamicObstacle has ellipsoidal shape
        ObstacleBase(double _radius, double _max_acc, double _downwash)
        : radius(_radius), max_acc(_max_acc), downwash(_downwash) {

        }

        dynamic_msgs::Obstacle getObstacle(double t){
            dynamic_msgs::Obstacle obstacle = getObstacle_impl(t);
//            if(type == "static"){
//                obstacle.type = ObstacleType::STATICOBSTACLE;
//            }
//            else{
                obstacle.type = ObstacleType::DYNAMICOBSTACLE;
//            }
            obstacle.max_acc = max_acc;
            obstacle.radius = radius;
            obstacle.downwash = downwash;
            return obstacle;
        }

        virtual dynamic_msgs::Obstacle getObstacle_impl(double t) {
            dynamic_msgs::Obstacle obstacle;
            return obstacle;
        }

        double getRadius() const{
            return radius;
        }

        double getMaxAcc() const{
            return max_acc;
        }

        std::string getType() const{
            return type;
        }

        double getDownwash() const{
            return downwash;
        }

        void setRadius(double _radius) {
            radius = _radius;
        }

    protected:
        std::string type;
        double radius;
        double max_acc;
        double downwash;
    };

    class SpinObstacle : public ObstacleBase {
    public:
        SpinObstacle(geometry_msgs::PoseStamped _axis, geometry_msgs::Point _start, double _radius, double _speed,
                     double _max_acc, double _downwash)
                : axis(std::move(_axis)), start(_start), speed(_speed), ObstacleBase(_radius, _max_acc, _downwash) {
            type = "spin";
        }

        dynamic_msgs::Obstacle getObstacle_impl(double t) override {
            return getSpinObstacle(t);
        }

    private:
        geometry_msgs::PoseStamped axis;
        geometry_msgs::Point start;
        double speed;

        dynamic_msgs::Obstacle getSpinObstacle(double t) {
            dynamic_msgs::Obstacle spinObstacle;
            Eigen::Vector3d a(start.x - axis.pose.position.x, start.y - axis.pose.position.y,
                              start.z - axis.pose.position.z);
            Eigen::Vector3d n(axis.pose.orientation.x, axis.pose.orientation.y, axis.pose.orientation.z);
            n.normalize();
            Eigen::Vector3d r = a - a.dot(n) * n;
            double spin_radius = r.norm();
            double w = speed / spin_radius;
            double theta = w * t;

            if (max_acc < speed * speed / spin_radius){
                ROS_WARN("[Obstacle] max_acc is smaller than real acc!");
//                throw(PlanningReport::INVALIDARGS);
            }

            // position
            Eigen::Quaterniond q(cos(theta / 2), sin(theta / 2) * n.x(), sin(theta / 2) * n.y(),
                                 sin(theta / 2) * n.z());
            Eigen::Quaterniond p(0, a.x(), a.y(), a.z());
            p = q * p * q.inverse();
            spinObstacle.pose.position.x = axis.pose.position.x + p.x();
            spinObstacle.pose.position.y = axis.pose.position.y + p.y();
            spinObstacle.pose.position.z = axis.pose.position.z + p.z();

            // velocity
            theta = M_PI / 2;
            Eigen::Quaterniond q_vel(cos(theta / 2), sin(theta / 2) * n.x(), sin(theta / 2) * n.y(),
                                     sin(theta / 2) * n.z());
            p = q_vel * p * q_vel.inverse();
            spinObstacle.velocity.linear.x = w * p.x();
            spinObstacle.velocity.linear.y = w * p.y();
            spinObstacle.velocity.linear.z = w * p.z();

            return spinObstacle;
        }
    };

    class StraightObstacle : public ObstacleBase {
    public:
        StraightObstacle() : ObstacleBase(0, 0, 1) {
            type = "straight";
            speed = 0;
            flight_time = 0;
        }

        StraightObstacle(geometry_msgs::Point _start, geometry_msgs::Point _goal, double _radius, double _speed,
                         double _max_acc, double _downwash)
                : start(_start), goal(_goal), speed(_speed), ObstacleBase(_radius, _max_acc, _downwash) {
            type = "straight";

            Eigen::Vector3d delta(goal.x - start.x, goal.y - start.y, goal.z - start.z);
            Eigen::Vector3d n = delta.normalized();
            v = speed * n;
            double distance = delta.norm();
            flight_time = distance / speed;
        }

        dynamic_msgs::Obstacle getObstacle_impl(double t) override {
            return getStraightObstacle(t);
        }

    private:
        geometry_msgs::Point start, goal;
        double speed;
        Eigen::Vector3d v;
        double flight_time;

        dynamic_msgs::Obstacle getStraightObstacle(double t) {
            dynamic_msgs::Obstacle straightObstacle;

            // position
            if(t < flight_time) {
                straightObstacle.pose.position.x = start.x + v(0) * t;
                straightObstacle.pose.position.y = start.y + v(1) * t;
                straightObstacle.pose.position.z = start.z + v(2) * t;
                straightObstacle.velocity.linear.x = v(0);
                straightObstacle.velocity.linear.y = v(1);
                straightObstacle.velocity.linear.z = v(2);
            }
            else{
                straightObstacle.pose.position = goal;
                straightObstacle.velocity.linear.x = 0;
                straightObstacle.velocity.linear.y = 0;
                straightObstacle.velocity.linear.z = 0;
            }
            return straightObstacle;
        }
    };

    class PatrolObstacle : public ObstacleBase {
    public:
        PatrolObstacle(std::vector<geometry_msgs::Point> _waypoints, double _radius, double _speed, double _max_acc, double _downwash)
                : waypoints(std::move(_waypoints)), speed(_speed), ObstacleBase(_radius, _max_acc, _downwash) {
            type = "multisim_patrol";
            flight_time.resize(waypoints.size());
            straightObstacles.resize(waypoints.size());
            for (int i = 0; i < waypoints.size(); i++) {
                double distance;
                if (i == waypoints.size() - 1) {
                    Eigen::Vector3d delta(waypoints[0].x - waypoints[i].x, waypoints[0].y - waypoints[i].y,
                                          waypoints[0].z - waypoints[i].z);
                    distance = delta.norm();
                    flight_time[i] = distance / speed;
                    straightObstacles[i] = StraightObstacle(waypoints[i], waypoints[0],
                                                            radius, speed, max_acc, downwash);
                } else {
                    Eigen::Vector3d delta(waypoints[i + 1].x - waypoints[i].x, waypoints[i + 1].y - waypoints[i].y,
                                          waypoints[i + 1].z - waypoints[i].z);
                    distance = delta.norm();
                    flight_time[i] = distance / speed;
                    straightObstacles[i] = StraightObstacle(waypoints[i], waypoints[i + 1],
                                                            radius, speed, max_acc, downwash);
                }
            }
        }

        dynamic_msgs::Obstacle getObstacle_impl(double t) override {
            return getPatrolObstacle(t);
        }

    private:
        std::vector<geometry_msgs::Point> waypoints;
        std::vector<double> flight_time;
        std::vector<StraightObstacle> straightObstacles;
        double speed;

        dynamic_msgs::Obstacle getPatrolObstacle(double t) {
            dynamic_msgs::Obstacle patrolObstacle;

            double current_time = t;
            int current_idx = 0;
            while (current_time >= flight_time[current_idx]){
                current_time -= flight_time[current_idx];
                if(current_idx == waypoints.size() - 1){
                    current_idx = 0;
                }
                else{
                    current_idx++;
                }
            }


            patrolObstacle = straightObstacles[current_idx].getObstacle(current_time);
            return patrolObstacle;
        }
    };

    //TODO: cannot used in oracle prediction
    class ChasingObstacle : public ObstacleBase {
    public:
        ChasingObstacle() : ObstacleBase(0, 0, 1) {
            type = "chasing";
            t_last_called = 0;
            max_vel = 0;
            gamma_target = 0;
            gamma_obs = 0;
        }

        ChasingObstacle(const dynamic_msgs::Obstacle& _start_state, double _radius, double _max_vel, double _max_acc,
                        double _gamma_target, double _gamma_obs, double _downwash)
                : ObstacleBase(_radius, _max_acc, _downwash) {
            type = "chasing";
            current_state.pose.position = _start_state.pose.position;
            current_state.velocity.linear.x = 0;
            current_state.velocity.linear.y = 0;
            current_state.velocity.linear.z = 0;
            max_vel = _max_vel;
            gamma_target = _gamma_target;
            gamma_obs = _gamma_obs;
            goal_point = current_state.pose.position;
            t_last_called = 0;
        }

        void setGoalPoint(const geometry_msgs::Point& _goal_point) {
            goal_point = _goal_point;
        }

        void setObstacles(const std::vector<dynamic_msgs::Obstacle>& _obstacles){
            obstacles = _obstacles;
        }

        dynamic_msgs::Obstacle getObstacle_impl(double t) override {
            return getChasingObstacle(t);
        }

    private:
        dynamic_msgs::Obstacle current_state;
        double t_last_called, max_vel, gamma_target, gamma_obs;
        geometry_msgs::Point goal_point;
        std::vector<dynamic_msgs::Obstacle> obstacles;
        int chasing_target_id;

        dynamic_msgs::Obstacle getChasingObstacle(double t) {
            dynamic_msgs::Obstacle chasingObstacle;
            Eigen::Vector3d delta_goal(goal_point.x - current_state.pose.position.x,
                                       goal_point.y - current_state.pose.position.y,
                                       goal_point.z - current_state.pose.position.z);

            Eigen::Vector3d a = gamma_target * delta_goal;
            Eigen::Vector3d v(current_state.velocity.linear.x,
                              current_state.velocity.linear.y,
                              current_state.velocity.linear.z);
            double dt = t - t_last_called;

            for(const auto & obstacle : obstacles){
                Eigen::Vector3d delta_obstacle(obstacle.pose.position.x - current_state.pose.position.x,
                                               obstacle.pose.position.y - current_state.pose.position.y,
                                               obstacle.pose.position.z - current_state.pose.position.z);
                double dist_to_obs = delta_obstacle.norm();
                if(dist_to_obs < SP_EPSILON_FLOAT){
                    continue;
                }
                double Q_star = 2 * (radius + obstacle.radius); //TODO: parameterization
                if(dist_to_obs < Q_star) {
//                    a += gamma_obs * (1/Q_star - 1/dist_to_obs) / (1/(dist_to_obs * dist_to_obs)) * delta_obstacle.normalized();
                    a += gamma_obs * (1 - dist_to_obs/Q_star) * (1/(dist_to_obs * Q_star)) * (-delta_obstacle.normalized());
                }
            }

            if(a.norm() > (max_acc-0.01)){
                a = a / a.norm() * (max_acc-0.01);
            }
            v = v + a * dt;
            if(v.norm() > max_vel){ //TODO: parameterization
                v = v / v.norm() * max_vel;
            }

            //update current_state
            current_state.pose.position.x = current_state.pose.position.x + v(0) * dt;
            current_state.pose.position.y = current_state.pose.position.y + v(1) * dt;
            current_state.pose.position.z = current_state.pose.position.z + v(2) * dt;
            current_state.velocity.linear.x = v(0);
            current_state.velocity.linear.y = v(1);
            current_state.velocity.linear.z = v(2);

            chasingObstacle.pose.position = current_state.pose.position;
            chasingObstacle.velocity.linear = current_state.velocity.linear;
            chasingObstacle.radius = radius;

            t_last_called = t;
            return chasingObstacle;
        }
    };

    class GaussianObstacle : public ObstacleBase {
    public:
        GaussianObstacle() : ObstacleBase(0, 0, 1) {
            type = "gaussian";
            acc_history_horizon = 0;
        }

        GaussianObstacle(geometry_msgs::Point _start, double _radius,
                         geometry_msgs::Vector3 _initial_vel, double _max_vel,
                         double _stddev_acc, double _max_acc, double _acc_update_cycle,
                         double _downwash)
                : start(_start), initial_vel(_initial_vel), max_vel(_max_vel),
                  stddev_acc(_stddev_acc), max_acc(_max_acc), acc_update_cycle(_acc_update_cycle),
                  ObstacleBase(_radius, _max_acc, _downwash)
        {
            type = "gaussian";
            acc_history_horizon = 0;
            update_acc_history(10);
        }

        dynamic_msgs::Obstacle getObstacle_impl(double t) override {
            return getGaussianObstacle(t);
        }

        void set_acc_history(std::vector<Eigen::Vector3d> _acc_history){
            acc_history = std::move(_acc_history);
        }

    private:
        geometry_msgs::Point start;
        std::vector<Eigen::Vector3d> acc_history;
        geometry_msgs::Vector3 initial_vel;
        double max_vel;
        double stddev_acc, max_acc, acc_update_cycle;
        double acc_history_horizon;

        void update_acc_history(double desired_horizon){
            if(acc_history_horizon < desired_horizon){
                std::random_device rd;
                std::mt19937 generator(rd());
                std::normal_distribution<double> distribution(0.0, stddev_acc);
                int n = ceil((desired_horizon - acc_history_horizon) / acc_update_cycle);
                acc_history_horizon += n * acc_update_cycle;

                for(int i = 0; i < n; i++){
                    Eigen::Vector3d acc;
                    for(int j = 0; j < 3; j++){
                        acc(j) = distribution(generator);
                    }
                    if(acc.norm() > max_acc){
                       acc = acc.normalized() * max_acc;
                    }

                    acc_history.emplace_back(acc);
                }
            }
        }

        dynamic_msgs::Obstacle getGaussianObstacle(double t) {
            dynamic_msgs::Obstacle gaussianObstacle;
            gaussianObstacle.pose.position = start;
            gaussianObstacle.velocity.linear = initial_vel;

            // update acc history
            if(t >= acc_history_horizon){
                double horizon_update = 10.0;
                update_acc_history(acc_history_horizon + horizon_update);
                acc_history_horizon += horizon_update;
            }

            // backtrack to find current state
            int n = floor((t + SP_EPSILON_FLOAT) / acc_update_cycle);
            if(acc_history.size() < n + 1){
                ROS_ERROR("acc history size error");
            }
            Eigen::Vector3d v(initial_vel.x, initial_vel.y, initial_vel.z);
            Eigen::Vector3d v_next;
            double dt = acc_update_cycle;
            for(int i = 0; i < n + 1; i++){
                if(i == n){
                    dt = t - n * acc_update_cycle;
                }

                Eigen::Vector3d acc = acc_history[i];
                v_next = v + acc * dt;
                if(v_next.norm() > max_vel){
                    // if v is over max_vel then ignore acc
                    //TODO: better max vel
                    gaussianObstacle.pose.position.x += v(0) * dt;
                    gaussianObstacle.pose.position.y += v(1) * dt;
                    gaussianObstacle.pose.position.z += v(2) * dt;
                }
                else{
                    gaussianObstacle.pose.position.x += v(0) * dt + 0.5 * acc(0) * dt * dt;
                    gaussianObstacle.pose.position.y += v(1) * dt + 0.5 * acc(1) * dt * dt;
                    gaussianObstacle.pose.position.z += v(2) * dt + 0.5 * acc(2) * dt * dt;
                    gaussianObstacle.velocity.linear.x += acc(0) * dt;
                    gaussianObstacle.velocity.linear.y += acc(1) * dt;
                    gaussianObstacle.velocity.linear.z += acc(2) * dt;
                    v = v_next;
                }
            }

            return gaussianObstacle;
        }
    };

//    class StaticObstacle : public ObstacleBase {
//    public:
//        StaticObstacle() : ObstacleBase(0, 0, 1) {
//            type = "static";
//        }
//
//        StaticObstacle(geometry_msgs::Point _pose, geometry_msgs::Point _dimensions)
//                : pose(_pose), dimensions(_dimensions), ObstacleBase(0, 0, 1)
//        {
//            type = "static";
//        }
//
//        dynamic_msgs::Obstacle getObstacle_impl(double t) override {
//            return getStaticObstacle();
//        }
//
//    private:
//        geometry_msgs::Point pose;
//        geometry_msgs::Point dimensions;
//
//        dynamic_msgs::Obstacle getStaticObstacle() {
//            dynamic_msgs::Obstacle staticObstacle;
//            staticObstacle.pose.position = pose;
//
//            staticObstacle.dimensions.resize(3);
//            staticObstacle.dimensions[0] = dimensions.x;
//            staticObstacle.dimensions[1] = dimensions.y;
//            staticObstacle.dimensions[2] = dimensions.z;
//
//            staticObstacle.velocity.linear.x = 0;
//            staticObstacle.velocity.linear.y = 0;
//            staticObstacle.velocity.linear.z = 0;
//
//            staticObstacle.goal_point = pose;
//            staticObstacle.max_acc = 0;
//            staticObstacle.downwash = 1;
//            staticObstacle.id = -1;
//            staticObstacle.radius = -1;
//
//            return staticObstacle;
//        }
//    };

    class BernsteinObstacle : public ObstacleBase {
    public:
        BernsteinObstacle(std::vector<points_t> _control_points, std::vector<double> _time_segments,
                          double _radius, double _max_acc, double _downwash)
                : control_points(std::move(_control_points)), time_segments(_time_segments), ObstacleBase(_radius, _max_acc, _downwash) {
            type = "bernstein";

            M = control_points.size();
            n = control_points[0].size() - 1;
        }

        dynamic_msgs::Obstacle getObstacle_impl(double t) override {
            return getBernsteinObstacle(t);
        }

        dynamic_msgs::State getObstacleState(double t) {
            dynamic_msgs::State state;
            state = getStateFromControlPoints(control_points, t, M, n, time_segments);
            return state;
        }

    private:
        std::vector<points_t> control_points;
        std::vector<double> time_segments;
        int M, n;

        dynamic_msgs::Obstacle getBernsteinObstacle(double t) {
            dynamic_msgs::Obstacle bernsteinObstacle;
            dynamic_msgs::State state;
            state = getStateFromControlPoints(control_points, t, M, n, time_segments);
            bernsteinObstacle.pose = state.pose;
            bernsteinObstacle.velocity = state.velocity;

            return bernsteinObstacle;
        }
    };
}
