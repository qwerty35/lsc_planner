#ifndef LSC_PLANNER_MAP_MANAGER_H
#define LSC_PLANNER_MAP_MANAGER_H

#include <ros/ros.h>
#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <util.hpp>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


namespace DynamicPlanning {
    class MapManager {
    public:
        MapManager(const ros::NodeHandle& nh, const Param& param, const Mission& mission, int agent_id);

        void publish() const;

        void updateLocalMap(const point3d& agent_position);

        void mergeMapCallback(const octomap_msgs::Octomap& msg_merge_map);

        void setGlobalMap(const sensor_msgs::PointCloud2& global_map);

        sensor_msgs::PointCloud2 getVirtualSensorInput(const point3d& agent_position);

        [[nodiscard]] octomap_msgs::Octomap getOctomapMsg() const;

        [[nodiscard]] std::shared_ptr<octomap::OcTree> getOctomap() const;

        [[nodiscard]] std::shared_ptr<DynamicEDTOctomap> getDistmap() const;

    private:
        Param param;
        Mission mission;

        pcl::PointCloud<pcl::PointXYZ> cloud_all_map;
        pcl::search::KdTree<pcl::PointXYZ> kdtreeGlobalMap;
        bool has_global_map;

        std::string agent_frame_id, world_frame_id;

        ros::NodeHandle nh;
        ros::Publisher pub_sensor_map;

        std::shared_ptr<octomap::OcTree> octree_ptr;
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;

        void updateGlobalMap(const sensor_msgs::PointCloud2& msg_global_map);

        void updateVirtualSensorInput(const point3d& agent_position);
    };
}

#endif //LSC_PLANNER_VIRTUAL_SENSOR_H
