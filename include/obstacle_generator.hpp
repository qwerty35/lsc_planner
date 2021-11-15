#pragma once

#include <fstream>
#include <iostream>
#include <math.h>
#include <sp_const.hpp>
#include <mission.hpp>
#include <obstacle.hpp>
#include <random>


//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_msgs/Trajectory.h>
#include <dynamic_msgs/ObstacleArray.h>

namespace DynamicPlanning {
    class ObstacleGenerator {
    public:
        ObstacleGenerator(const ros::NodeHandle &_nh, const Mission& _mission) : nh(_nh), mission(_mission) {
            pubs_obstacles.resize(mission.qn);
            for(int qi = 0; qi < mission.qn; qi++){
                pubs_obstacles[qi] = nh.advertise<dynamic_msgs::ObstacleArray>("/mav" + std::to_string(qi) + "_obstacles_state", 1);
            }
            pub_obstacle_collision_model = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_collision_model",
                                                                                         1);
            start_time = ros::Time::now();
            obstacles.resize(mission.on);
        }

        void update(double t, double observer_stddev){
            std::vector<geometry_msgs::Point> empty_vector;
            empty_vector.resize(mission.on);
            updateObstacles(t, observer_stddev, empty_vector);

            //update obstacle msg and add measurement error
            updateObstaclesMsg(t, observer_stddev);
        }

        void update(double t, double observer_stddev, const std::vector<geometry_msgs::Point>& chasing_goal_points){
            updateObstacles(t, observer_stddev, chasing_goal_points);

            //update obstacle msg and add measurement error
            updateObstaclesMsg(t, observer_stddev);
        }

        void update(double t, double observer_stddev, const std::vector<dynamic_msgs::Obstacle>& _obstacles){
            obstacles = _obstacles;

            //update obstacle msg and add measurement error
            updateObstaclesMsg(t, observer_stddev);
        }

        void publishForAgent(int qi) {
            publishObstaclesForAgent(qi);
        }

        void publish() {
            publishObstacles();
        }

        int getNumObs() const {
            return mission.on;
        }

        // get obstacle states which contain measurement error
        dynamic_msgs::Obstacle getObstacle(int oi){
            return msg_obstacles.obstacles[oi];
        }

        // safety_margin = distance to obstacle / (radius + obs_size) - 1
        std::vector<double> getSafetyMargin(int mav_id, const dynamic_msgs::State &current_state) {
            std::vector<double> safety_margin;
            safety_margin.resize(mission.on);
            for (int oi = 0; oi < mission.on; oi++) {
                safety_margin[oi] = std::sqrt(
                        pow(current_state.pose.position.x - obstacles[oi].pose.position.x, 2) +
                        pow(current_state.pose.position.y - obstacles[oi].pose.position.y, 2) +
                        pow(current_state.pose.position.z - obstacles[oi].pose.position.z, 2)) /
                                            (mission.agents[mav_id].radius + mission.obstacles[oi]->getRadius()) - 1;
            }

            return safety_margin;
        }

        void resetStartTime(ros::Time _start_time){
            start_time = _start_time;
        }

    private:
        ros::NodeHandle nh;
        std::vector<ros::Publisher> pubs_obstacles;
        ros::Publisher pub_obstacle_collision_model;

        Mission mission;
        ros::Time start_time;
        std::vector<dynamic_msgs::Obstacle> obstacles; // does not contain measurement error
        dynamic_msgs::ObstacleArray msg_obstacles; // contain measurement error

        void updateObstacles(double t, double observer_stddev, const std::vector<geometry_msgs::Point>& chasing_points){
            // if obstacle is chasing then update target and other obstacles information
            for (int oi = 0; oi < mission.on; oi++) {
                if(mission.obstacles[oi]->getType() == "chasing"){
                    std::shared_ptr<ChasingObstacle> chasing_obstacle_ptr = std::static_pointer_cast<ChasingObstacle>(mission.obstacles[oi]);
                    chasing_obstacle_ptr->setGoalPoint(chasing_points[oi]);
                    std::vector<dynamic_msgs::Obstacle> obstacles_prev_step = obstacles;
                    obstacles_prev_step.erase(obstacles_prev_step.begin() + oi);
                    chasing_obstacle_ptr->setObstacles(obstacles_prev_step);
                }
            }

            //update obstacles
            for(int oi = 0; oi < mission.on; oi++){
                obstacles[oi] = mission.obstacles[oi]->getObstacle(t);
            }
        }

        void updateObstaclesMsg(double t, double observer_stddev){
            msg_obstacles.obstacles.resize(obstacles.size());
            msg_obstacles.start_time = start_time;
            msg_obstacles.header.stamp = start_time + ros::Duration(t);

            std::random_device rd;
            std::mt19937 generator(rd());
            std::normal_distribution<double> distribution(0, observer_stddev);
            for (int oi = 0; oi < obstacles.size(); oi++) {
                // obstacle msg
                msg_obstacles.obstacles[oi] = obstacles[oi];
                msg_obstacles.obstacles[oi].pose.position.x = obstacles[oi].pose.position.x + distribution(generator);
                msg_obstacles.obstacles[oi].pose.position.y = obstacles[oi].pose.position.y + distribution(generator);
                msg_obstacles.obstacles[oi].pose.position.z = obstacles[oi].pose.position.z + distribution(generator);
//                msg_obstacles.obstacles[oi].velocity.linear.x = obstacles[oi].velocity.linear.x;
//                msg_obstacles.obstacles[oi].velocity.linear.y = obstacles[oi].velocity.linear.y;
//                msg_obstacles.obstacles[oi].velocity.linear.z = obstacles[oi].velocity.linear.z;
//                msg_obstacles.obstacles[oi].radius = obstacles[oi].radius;
//                msg_obstacles.obstacles[oi].max_acc = obstacles[oi].max_acc;
//                msg_obstacles.obstacles[oi].type = obstacles[oi].type;
//                msg_obstacles.obstacles[oi].downwash = obstacles[oi].downwash;
            }
        }

        void publishObstaclesForAgent(int qi) {
            pubs_obstacles[qi].publish(msg_obstacles);
        }

        void publishObstacles() {
            visualization_msgs::MarkerArray msg_obstacle_collision_model;
            msg_obstacle_collision_model.markers.clear();

            for (int oi = 0; oi < mission.on; oi++) {
                // obstacle collision model
                visualization_msgs::Marker mk_obs_true_position;
                mk_obs_true_position.header.frame_id = "world"; //TODO: world frame id
                mk_obs_true_position.ns = "true";
                mk_obs_true_position.id = oi;

                mk_obs_true_position.action = visualization_msgs::Marker::ADD;
//                mk_obs_true_position.lifetime = ros::Duration(1.0);
                mk_obs_true_position.pose.orientation.x = 0;
                mk_obs_true_position.pose.orientation.y = 0;
                mk_obs_true_position.pose.orientation.z = 0;
                mk_obs_true_position.pose.orientation.w = 1.0;
                mk_obs_true_position.pose.position.x = obstacles[oi].pose.position.x;
                mk_obs_true_position.pose.position.y = obstacles[oi].pose.position.y;
                mk_obs_true_position.pose.position.z = obstacles[oi].pose.position.z;

//                if(mission.obstacles[oi]->getType() == "static"){
//                    mk_obs_true_position.type = visualization_msgs::Marker::CUBE;
//                    mk_obs_true_position.scale.x = 2 * obstacles[oi].dimensions[0];
//                    mk_obs_true_position.scale.y = 2 * obstacles[oi].dimensions[1];
//                    mk_obs_true_position.scale.z = 2 * obstacles[oi].dimensions[2];
//                }
//                else
                {
                    mk_obs_true_position.type = visualization_msgs::Marker::SPHERE;
                    mk_obs_true_position.scale.x = 2 * obstacles[oi].radius;
                    mk_obs_true_position.scale.y = 2 * obstacles[oi].radius;
                    mk_obs_true_position.scale.z = 2 * obstacles[oi].radius * obstacles[oi].downwash;
                }

                mk_obs_true_position.color.a = 0.5;
                mk_obs_true_position.color.r = 0;
                mk_obs_true_position.color.g = 0;
                mk_obs_true_position.color.b = 0;
                msg_obstacle_collision_model.markers.emplace_back(mk_obs_true_position);

                visualization_msgs::Marker mk_obs_observed_position = mk_obs_true_position;
                mk_obs_observed_position.ns = "observed";
                mk_obs_observed_position.id = oi;
                mk_obs_observed_position.pose.position.x = msg_obstacles.obstacles[oi].pose.position.x;
                mk_obs_observed_position.pose.position.y = msg_obstacles.obstacles[oi].pose.position.y;
                mk_obs_observed_position.pose.position.z = msg_obstacles.obstacles[oi].pose.position.z;
                msg_obstacle_collision_model.markers.emplace_back(mk_obs_observed_position);
            }

            pub_obstacle_collision_model.publish(msg_obstacle_collision_model);
        }
    };
}