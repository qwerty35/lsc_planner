#pragma once
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/ColorRGBA.h>
#include <string>
#include <octomap/OcTree.h>
#include <obstacle.hpp>
#include <random>
#include <csv_reader.hpp>
#include <util.hpp>
#include <experimental/filesystem>

using namespace rapidjson;
namespace fs = std::experimental::filesystem;

namespace DynamicPlanning {
    class Mission {
    public:
        size_t qn; // the number of quadrotors
        size_t on; // the number of obstacles
        std::map<std::string, Agent> quadrotor_map;
        std::vector<Agent> agents;
        std::vector<std::shared_ptr<ObstacleBase>> obstacles;
        point3d world_min, world_max;
        std::vector<std_msgs::ColorRGBA> color;
        std::vector<std::string> mission_file_names;
        std::vector<std::string> world_file_names;
        std::string current_mission_file_name;
        std::string current_world_file_name;

        explicit Mission(const ros::NodeHandle &nh);
        static Document parseMissionFile(const std::string& file_name);
        bool loadMission(double max_noise, int world_dimension, double world_z_2d = 1.0, int mission_idx = 0);
        bool changeMission(const std::string& mission_file_name, double max_noise, int world_dimension, double world_z_2d = 1.0);
        bool readMissionFile(double max_noise, int world_dimension, double world_z_2d);
        void addAgent(const Agent& agent);
        void addObstacle(const std::shared_ptr<ObstacleBase>& obstacle_ptr);
        void addNoise(double max_noise, int dimension);
        void initializeAgentColor();
        void saveMission(const std::string& addr) const; // save current mission to reproduce the same result
    };
}