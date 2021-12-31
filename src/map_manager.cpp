#include "map_manager.hpp"

namespace DynamicPlanning {
    MapManager::MapManager(const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission, int agent_id)
        : nh(_nh), param(_param), mission(_mission) {
        agent_frame_id = "mav" + std::to_string(agent_id);
        world_frame_id = param.world_frame_id;

        has_global_map = false;

        octree_ptr = std::make_shared<octomap::OcTree>(param.world_resolution);
        distmap_ptr = std::make_shared<DynamicEDTOctomap>(1.0, octree_ptr.get(),
                                                          mission.world_min, mission.world_max, false);

        std::string prefix = "/mav" + std::to_string(agent_id);
        pub_sensor_map = nh.advertise<octomap_msgs::Octomap>(prefix + "/local_octomap", 1);
    }

    void MapManager::publish() const {
        if(!has_global_map){
            return;
        }

        if(not param.world_use_global_map){
            pub_sensor_map.publish(getOctomapMsg());
        }
    }

    octomap_msgs::Octomap MapManager::getOctomapMsg() const{
        octomap_msgs::Octomap msg_local_octomap;
        octomap_msgs::fullMapToMsg(*octree_ptr, msg_local_octomap);
        msg_local_octomap.header.frame_id = world_frame_id;
        msg_local_octomap.header.stamp = ros::Time::now();
        return msg_local_octomap;
    }

    std::shared_ptr<octomap::OcTree> MapManager::getOctomap() const {
        return octree_ptr;
    }

    std::shared_ptr<DynamicEDTOctomap> MapManager::getDistmap() const {
        return distmap_ptr;
    }

    void MapManager::setGlobalMap(const sensor_msgs::PointCloud2& msg_global_map) {
        if(has_global_map){
            return;
        }

        if(param.world_use_global_map){
            updateGlobalMap(msg_global_map);
            distmap_ptr = std::make_shared<DynamicEDTOctomap>(1.0, octree_ptr.get(),
                                                              mission.world_min, mission.world_max,
                                                              false);
            distmap_ptr->update();
        }
        else{
            pcl::PointCloud<pcl::PointXYZ> cloud_input;
            pcl::fromROSMsg(msg_global_map, cloud_input);

            pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
            voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
            voxel_sampler.setInputCloud(cloud_input.makeShared());
            voxel_sampler.filter(cloud_all_map);

            kdtreeGlobalMap.setInputCloud(cloud_all_map.makeShared());
        }

        has_global_map = true;
    }

    void MapManager::updateGlobalMap(const sensor_msgs::PointCloud2& msg_global_map){
        pcl::PointCloud<pcl::PointXYZ> global_map;
        pcl::fromROSMsg(msg_global_map, global_map);
        octomap::Pointcloud octomap_pointcloud;
        for (const auto& point : global_map.points) {
            octomap_pointcloud.push_back(point3d(point.x, point.y, point.z));
        }

        octree_ptr->insertPointCloud(octomap_pointcloud, point3d(0,0,0));
    }

    void MapManager::updateLocalMap(const point3d& agent_position){
        if(not has_global_map){
            return;
        }

        //sensor intput
        updateVirtualSensorInput(agent_position);
        distmap_ptr->update();
    }

    void MapManager::updateVirtualSensorInput(const point3d& agent_position){
        sensor_msgs::PointCloud2 msg_sensor_map = getVirtualSensorInput(agent_position);

        pcl::PointCloud<pcl::PointXYZ> local_sensor_map;
        pcl::fromROSMsg(msg_sensor_map, local_sensor_map);

        pcl::PointCloud<pcl::PointXYZ> sensor_map;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << agent_position.x(), agent_position.y(), agent_position.z();
        pcl::transformPointCloud(local_sensor_map, sensor_map, transform);

        octomap::Pointcloud sensor_octomap;
        for (const auto& point : sensor_map.points) {
            if(isnan(point.x)){
                continue;
            }
            if(point.z < -1.0){
                continue;
            }
            sensor_octomap.push_back(point3d(point.x, point.y, point.z));
        }

        octree_ptr->insertPointCloud(sensor_octomap, agent_position, param.sensor_range);
    }

    // global frame
//    void VirtualSensor::publishSensorMap(const point3d& agent_position) {
//        // local map
//        if(!has_global_map) {
//            return;
//        }
//
//        sensor_msgs::PointCloud2 msg_sensor_map;
//        pcl::PointCloud<pcl::PointXYZ> sensor_map;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr asdf = sensor_map.makeShared();
//
//        std::vector<int> pointIdxRadiusSearch;
//        std::vector<float> pointRadiusSquaredDistance;
//
//        sensor_map.points.clear();
//        pointIdxRadiusSearch.clear();
//        pointRadiusSquaredDistance.clear();
//        pcl::PointXYZ searchPoint(agent_position.x(),
//                                  agent_position.y(),
//                                  agent_position.z());
//
//        pcl::PointXYZ pt;
//        if (kdtreeLocalMap.radiusSearch(searchPoint, sensor_range,
//                                        pointIdxRadiusSearch,
//                                        pointRadiusSquaredDistance) > 0) {
//            for (int idx: pointIdxRadiusSearch) {
//                pt = cloud_all_map.points[idx];
//                sensor_map.points.push_back(pt);
//            }
//        }
//
//        if(has_merge_map){
//            sensor_map.points.insert( sensor_map.points.end(), merge_map.points.begin(), merge_map.points.end());
//            has_merge_map = false;
//        }
//
//        sensor_map.width = sensor_map.points.size();
//        sensor_map.height = 1;
//        sensor_map.is_dense = true;
//
//        pcl::toROSMsg(sensor_map, msg_sensor_map);
//        msg_sensor_map.header.frame_id = frame_id;
//        pub_sensor_map.publish(msg_sensor_map);
//    }

    sensor_msgs::PointCloud2 MapManager::getVirtualSensorInput(const point3d& agent_position) {
        sensor_msgs::PointCloud2 msg_sensor_map;
        pcl::PointCloud<pcl::PointXYZ> sensor_map;
        sensor_map.points.clear();

        // Radius search
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        pcl::PointXYZ searchPoint(agent_position.x(),
                                  agent_position.y(),
                                  agent_position.z());

        pcl::PointXYZ pt;
        if (kdtreeGlobalMap.radiusSearch(searchPoint, param.sensor_range,
                                         pointIdxRadiusSearch,
                                         pointRadiusSquaredDistance) > 0) {
            for (int idx: pointIdxRadiusSearch) {
                pt = cloud_all_map.points[idx];
                sensor_map.points.push_back(pt);
            }
        }

        // Transform by the agent pose
        pcl::PointCloud<pcl::PointXYZ> local_sensor_map;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << -agent_position.x(), -agent_position.y(), -agent_position.z();
        pcl::transformPointCloud(sensor_map, local_sensor_map, transform);

        local_sensor_map.width = local_sensor_map.points.size();
        local_sensor_map.height = 1;
        local_sensor_map.is_dense = true;

        pcl::toROSMsg(local_sensor_map, msg_sensor_map);
        msg_sensor_map.header.frame_id = agent_frame_id;
        msg_sensor_map.header.stamp = ros::Time::now();
        return msg_sensor_map;
    }

    void MapManager::mergeMapCallback(const octomap_msgs::Octomap& msg_merge_map){
        if(param.world_use_global_map){
            return;
        }

        auto* merge_octree_ptr = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(msg_merge_map));

        // Expand tree2 so we search all nodes
        merge_octree_ptr->expand();

        // traverse nodes in tree2 to add them to tree1
        for (octomap::OcTree::leaf_iterator it = merge_octree_ptr->begin_leafs();
             it != merge_octree_ptr->end_leafs(); ++it) {

            // find if the current node maps a point in map1
            point3d point = it.getCoordinate();
            octomap::OcTreeNode *nodeIn1 = octree_ptr->search(point);
            if (nodeIn1 != nullptr) {
                // Add the probability of tree2 node to the found node
                octomap::OcTreeKey nodeKey = octree_ptr->coordToKey(point);
                octree_ptr->updateNode(nodeKey, it->getLogOdds());
            } else {
                // Create a new node and set the probability from tree2
                octomap::OcTreeNode *newNode = octree_ptr->updateNode(point, true);
                newNode->setLogOdds(it->getLogOdds());
            }
        }

        delete merge_octree_ptr;
    }
}