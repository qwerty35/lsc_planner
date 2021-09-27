#include <multi_sync_simulator.hpp>
#include <multi_sync_replayer.hpp>

using namespace DynamicPlanning;

int main(int argc, char* argv[]){
    ROS_INFO("Multi Sync Simulator");
    ros::init (argc, argv, "multi_sync_simulator_node");
    ros::NodeHandle nh( "~" );

    // Load ROS parameters
    Param param;
    if(!param.initialize(nh)){
        ROS_ERROR("[MultiSyncSimulator] Invalid parameter in launch file");
        return -1;
    }

    // Replay mode
    // Activated when the argument of launch file is <arg name="replay" default="true" />
    if (param.multisim_replay) {
        Mission mission;
        if (not mission.initialize(param.mission_file_names[0], param.multisim_max_noise,
                                   param.world_dimension, param.world_z_2d)) {
            ROS_ERROR("[MultiSyncSimulator] Invalid mission");
            return -1;
        }

        MultiSyncReplayer multi_sync_replayer(nh, param, mission);
        ros::Rate rate(50);
        ros::Time replay_start_time = ros::Time::now();
        multi_sync_replayer.readCSVFile(param.multisim_replay_file_name);
        while (ros::ok()) {
            multi_sync_replayer.replay((ros::Time::now() - replay_start_time).toSec());
            ros::spinOnce();
            rate.sleep();
        }

        return 1;
    }

    // Planner mode
    // Activated when the argument of launch file is <arg name="replay" default="false" />
    for(int si = 0; si < param.mission_file_names.size(); si++){
        // Load world from octomap
        std::string world_file_name = "none";
        if(param.world_use_octomap){
            if(param.world_file_names.size() == param.mission_file_names.size()){
                world_file_name = param.world_file_names[si];
            }
            else{
                ROS_WARN_STREAM_ONCE("The number of world file is not match to the number of mission file, use "
                                     << param.world_file_names[0]);
                world_file_name = param.world_file_names[0];
            }
        }

        // Load mission from json file
        std::string mission_file_name = param.mission_file_names[si];
        Mission mission;
        if (not mission.initialize(mission_file_name, param.multisim_max_noise, param.world_dimension,
                                   param.world_z_2d, world_file_name)) {
            ROS_ERROR("[MultiSyncSimulator] Invalid mission");
            return -1;
        }
        if (mission.qn == 0) {
            ROS_ERROR("[MultiSyncSimulator] Invalid mission, there is no agent");
            return -1;
        }

        // Initialize simulator
        MultiSyncSimulator multi_sync_simulator(nh, param, mission);

        // Run simulator
        multi_sync_simulator.run();
    }
}