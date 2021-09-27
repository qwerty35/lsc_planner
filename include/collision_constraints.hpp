#ifndef LSC_PLANNER_COLLISION_CONSTRAINTS_HPP
#define LSC_PLANNER_COLLISION_CONSTRAINTS_HPP

#include <vector>
#include <octomap/octomap_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_msgs/CollisionConstraint.h>
#include <sp_const.hpp>
#include <param.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <utility>

namespace DynamicPlanning {
    // Linear Safe Corridor
    // LSC = {c \in R^3 | (c - c_obs).dot(normal_vector) - d > 0}
    class LSC{
    public:
        LSC() = default;
        LSC(const octomap::point3d& obs_control_point,
            const octomap::point3d& normal_vector,
            double d);
        visualization_msgs::Marker convertToMarker(double agent_radius) const;

        octomap::point3d obs_control_point;
        octomap::point3d normal_vector;
        double d = 0;
    };

    typedef std::vector<LSC> LSCs;

    class Box{
    public:
        Box() = default;
        Box(const octomap::point3d& box_min, const octomap::point3d& box_max);

        LSCs convertToLSCs(int dim) const;
        visualization_msgs::Marker convertToMarker(double agent_radius) const;

        bool isPointInBox(const octomap::point3d& point) const;
        bool isPointsInTwoBox(const std::vector<octomap::point3d>& points, const Box& other_sfc) const;
        bool isSuperSetOfConvexHull(const std::vector<octomap::point3d>& convex_hull) const;
        bool isLineOnBoundary(const Line& line) const;
        bool intersectWith(const Box& other_sfc) const;

        Box unify(const Box& other_sfc) const;
        Box intersection(const Box& other_sfc) const;

        std::vector<octomap::point3d> getVertices() const;
        lines_t getEdges() const;
        octomap::point3d getBoxMin() const;
        octomap::point3d getBoxMax() const;

    private:
        octomap::point3d box_min;
        octomap::point3d box_max;
    };

    class SFC{
    public:
        Box box;
        LSCs lscs;

        Box inter_box; // visualization purpose
        int box_library_idx1, box_library_idx2; // debugging purpose

        LSCs convertToLSCs(int dim) const;
        bool update(const Box& box); //return whether success
        bool update(const std::vector<octomap::point3d>& convex_hull, const Box& box1, const Box& box2); //return whether success
        bool update(const std::vector<octomap::point3d>& convex_hull, const Box& box1, const Box& box2, double& dist_to_convex_hull); //return whether success
    };


    typedef std::vector<std::vector<std::vector<LSC>>> RSFCs; // [obs_idx][segment_idx][control_point_idx]
    typedef std::vector<SFC> SFCs; // [segment_idx]
    typedef std::vector<Box> Boxes;

    class CollisionConstraints {
    public:
        CollisionConstraints() = default;

        void initialize(int N_obs, int M, int n, double dt, std::set<int> obs_slack_indices);

        LSC getLSC(int oi, int m, int i) const;
        SFC getSFC(int m) const;
        size_t getObsSize() const;
        std::set<int> getSlackIndices() const;

        void setLSC(int oi, int m,
                    const std::vector<octomap::point3d>& obs_control_points,
                    const octomap::point3d& normal_vector,
                    const std::vector<double>& ds);
        void setLSC(int oi, int m,
                    const std::vector<octomap::point3d>& obs_control_points,
                    const octomap::point3d& normal_vector,
                    double d);
        void setSFC(int m, const SFC& sfc);
        void setSFC(int m, const Box& box);

        visualization_msgs::MarkerArray convertToMarkerArrayMsg(const std::vector<dynamic_msgs::Obstacle>& obstacles,
                                                                const std::vector<std_msgs::ColorRGBA>& colors,
                                                                int agent_id, double agent_radius) const;
        dynamic_msgs::CollisionConstraint convertToRawMsg(const std::vector<dynamic_msgs::Obstacle>& obstacles,
                                                          int planner_seq) const;

    private:
        RSFCs lscs;
        SFCs sfcs;

        std::set<int> obs_slack_indices;
        Boxes box_library;
        int N_obs, M, n;
        double dt;

        void setProperSFC(int m, const std::vector<octomap::point3d>& convex_hull);

        visualization_msgs::MarkerArray convertLSCsToMarkerArrayMsg(
                const std::vector<dynamic_msgs::Obstacle>& obstacles,
                const std::vector<std_msgs::ColorRGBA>& colors,
                double agent_radius) const;
        visualization_msgs::MarkerArray convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA& color,
                                                                    double agent_radius) const;
    };
}


#endif //LSC_PLANNER_COLLISION_CONSTRAINTS_HPP
