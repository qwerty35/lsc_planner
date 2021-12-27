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

using namespace rapidjson;

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
        std::string mission_file_name;
        std::string world_file_name;

        Mission();
        static Document readMissionFile(const std::string& file_name) ;
        bool initialize(const std::string& mission_file_name, double max_noise, int world_dimension,
                        double world_z_2d = 1.0, const std::string& world_file_name = "none");
        void generateCircleSwap(double circle_radius, int multisim_qn, double z_2d);
        Agent defaultAgent() const;
        void resetMission();
        void addAgent(const Agent& agent);
        void addObstacle(const std::shared_ptr<ObstacleBase>& obstacle_ptr);
        void addNoise(double max_noise, int dimension);
        void initializeAgentColor();
        void saveMission(const std::string& addr) const; // save current mission to reproduce the same result
    };
}