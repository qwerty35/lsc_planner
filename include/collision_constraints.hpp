#ifndef LSC_PLANNER_COLLISION_CONSTRAINTS_HPP
#define LSC_PLANNER_COLLISION_CONSTRAINTS_HPP

#include <vector>
#include <octomap/octomap_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_msgs/CollisionConstraint.h>
#include <sp_const.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <utility>

namespace DynamicPlanning {
    // Linear Safe Corridor
    // LSC = {c \in R^3 | (c - c_obs).dot(normal_vector) - d > 0}
    class LSC{
    public:
        LSC() = default;
        LSC(const point3d& obs_control_point,
            const point3d& normal_vector,
            double d);
        visualization_msgs::Marker convertToMarker(double agent_radius, const std::string& world_frame_id) const;

        point3d obs_control_point;
        point3d normal_vector;
        double d = 0;
    };

    typedef std::vector<LSC> LSCs;

    // Safe Flight Corridor
    // SFC = {c \in R^3 | box_min < c < box_max}
    class SFC{
    public:
        point3d box_min;
        point3d box_max;

        SFC() = default;
        SFC(const point3d& box_min, const point3d& box_max);

        [[nodiscard]] LSCs convertToLSCs(int dim) const;
        [[nodiscard]] visualization_msgs::Marker convertToMarker(double agent_radius,
                                                                 const std::string& world_frame_id) const;

        [[nodiscard]] bool isPointInSFC(const point3d& point) const;
        [[nodiscard]] bool isLineInSFC(const Line& line) const;
        [[nodiscard]] bool isSFCInBoundary(const point3d& world_min, const point3d& world_max, double margin) const;
        [[nodiscard]] bool isSuperSetOfConvexHull(const points_t& convex_hull) const;
        [[nodiscard]] bool intersectWith(const SFC& other_sfc) const;

        [[nodiscard]] SFC unify(const SFC& other_sfc) const;
        [[nodiscard]] SFC intersection(const SFC& other_sfc) const;

        [[nodiscard]] double distanceToPoint(const point3d& point) const;
        [[nodiscard]] double distanceToInnerPoint(const point3d& point) const;
        [[nodiscard]] double raycastFromInnerPoint(const point3d& point, const point3d& direction) const;
        [[nodiscard]] double raycastFromInnerPoint(const point3d& point, const point3d& direction,
                                                   point3d& surface_direction) const;

        [[nodiscard]] points_t getVertices() const;
        [[nodiscard]] lines_t getEdges() const;
    };

    typedef std::vector<std::vector<std::vector<LSC>>> RSFCs; // [obs_idx][segment_idx][control_point_idx]
    typedef std::vector<SFC> SFCs; // [segment_idx]


    class CollisionConstraints {
    public:
        CollisionConstraints(const Param& param, const Mission& mission);

        void initializeSFC(const point3d& agent_position, double radius);

        void initializeLSC(size_t N_obs);

        void updateSFCLibrary(const points_t& grid_path, double agent_radius);

        void generateFeasibleSFC(const point3d& last_point, const point3d& current_goal_position,
                                 const points_t& grid_path, double agent_radius);

        point3d findProperGoal(const point3d& last_point, const point3d& desired_goal, const points_t& grid_path);

        SFC findProperSFC(const point3d& start_point, const point3d& goal_point);

        // Getter
        [[nodiscard]] LSC getLSC(int oi, int m, int i) const;

        [[nodiscard]] SFC getSFC(int m) const;

        [[nodiscard]] size_t getObsSize() const;

        [[nodiscard]] std::set<int> getSlackIndices() const;

        // Setter
        void setDistmap(std::shared_ptr<DynamicEDTOctomap> distmap_ptr);

        void setObsSlackIndicies(const std::set<int>& obs_slack_indices);

        void setLSC(int oi, int m,
                    const points_t& obs_control_points,
                    const point3d& normal_vector,
                    const std::vector<double>& ds);

        void setLSC(int oi, int m,
                    const points_t& obs_control_points,
                    const point3d& normal_vector,
                    double d);

        void setSFC(int m, const SFC& sfc);

        // Converter
        [[nodiscard]] visualization_msgs::MarkerArray convertToMarkerArrayMsg(
                const std::vector<Obstacle>& obstacles,
                const std::vector<std_msgs::ColorRGBA>& colors,
                int agent_id, double agent_radius) const;
        [[nodiscard]] dynamic_msgs::CollisionConstraint convertToRawMsg(const std::vector<Obstacle>& obstacles,
                                                                        int planner_seq) const;

    private:
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;
        Mission mission;
        Param param;

        RSFCs lscs;
        SFCs sfcs;
        std::set<int> obs_slack_indices; //TODO: not updated yet
        int M = 0, n = 0;
        double dt = 0;
        SFCs sfc_library;

        [[nodiscard]] visualization_msgs::MarkerArray convertLSCsToMarkerArrayMsg(
                const std::vector<Obstacle>& obstacles,
                const std::vector<std_msgs::ColorRGBA>& colors,
                double agent_radius) const;

        [[nodiscard]] visualization_msgs::MarkerArray convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA& color,
                                                                                  double agent_radius) const;

        SFC expandSFCFromPoint(const point3d& point, double agent_radius);
        SFC expandSFCFromLine(const Line& line, double agent_radius);
        [[nodiscard]] std::vector<double> initializeBoxFromPoints(const points_t& points) const;
        bool isObstacleInSFC(const SFC& initial_sfc, double margin);
        bool isSFCInBoundary(const SFC& sfc, double margin);
        SFC expandSFC(const SFC& initial_sfc, double margin);
    };
}


#endif //LSC_PLANNER_COLLISION_CONSTRAINTS_HPP
