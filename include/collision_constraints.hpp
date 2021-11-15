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
        LSC(const point_t& obs_control_point,
            const point_t& normal_vector,
            double d);
        visualization_msgs::Marker convertToMarker(double agent_radius) const;

        point_t obs_control_point;
        point_t normal_vector;
        double d = 0;
    };

    typedef std::vector<LSC> LSCs;

    // Safe Flight Corridor
    // SFC = {c \in R^3 | box_min < c < box_max}
    class SFC{
    public:
        point_t box_min;
        point_t box_max;

        SFC() = default;
        SFC(const point_t& box_min, const point_t& box_max);

        [[nodiscard]] LSCs convertToLSCs(int dim) const;
        [[nodiscard]] visualization_msgs::Marker convertToMarker(double agent_radius) const;

        [[nodiscard]] bool isPointInSFC(const point_t& point) const;
        [[nodiscard]] bool isLineInSFC(const Line& line) const;
        [[nodiscard]] bool isSFCInBoundary(const point_t& world_min, const point_t& world_max, double margin) const;
        [[nodiscard]] bool isSuperSetOfConvexHull(const points_t& convex_hull) const;
        [[nodiscard]] bool intersectWith(const SFC& other_sfc) const;

        [[nodiscard]] SFC unify(const SFC& other_sfc) const;
        [[nodiscard]] SFC intersection(const SFC& other_sfc) const;

        [[nodiscard]] double distanceToPoint(const point_t& point) const;

        [[nodiscard]] points_t getVertices() const;
        [[nodiscard]] lines_t getEdges() const;
    };

    typedef std::vector<std::vector<std::vector<LSC>>> RSFCs; // [obs_idx][segment_idx][control_point_idx]
    typedef std::vector<SFC> SFCs; // [segment_idx]


    class CollisionConstraints {
    public:
        CollisionConstraints() = default;

        void initialize(std::shared_ptr<DynamicEDTOctomap> distmap_ptr, int N_obs, std::set<int> obs_slack_indices,
                        const Param& param, const Mission& mission);

        void initializeSFC(const point_t& agent_position, double radius);

        void generateFeasibleSFC(const point_t& last_point, const point_t& current_goal_position,
                                 const points_t& grid_path, double agent_radius);

        void updateSFCLibrary();

        // Getter
        LSC getLSC(int oi, int m, int i) const;

        SFC getSFC(int m) const;

        size_t getObsSize() const;

        std::set<int> getSlackIndices() const;

        // Setter
        void setLSC(int oi, int m,
                    const points_t& obs_control_points,
                    const point_t& normal_vector,
                    const std::vector<double>& ds);

        void setLSC(int oi, int m,
                    const points_t& obs_control_points,
                    const point_t& normal_vector,
                    double d);

        void setSFC(int m, const SFC& sfc);

        // Converter
        visualization_msgs::MarkerArray convertToMarkerArrayMsg(const std::vector<Obstacle>& obstacles,
                                                                const std::vector<std_msgs::ColorRGBA>& colors,
                                                                int agent_id, double agent_radius) const;
        dynamic_msgs::CollisionConstraint convertToRawMsg(const std::vector<Obstacle>& obstacles,
                                                          int planner_seq) const;

    private:
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;
        Mission mission;
        Param param;

        RSFCs lscs;
        SFCs sfcs;
        std::set<int> obs_slack_indices;
        int N_obs, M, n;
        double dt;
        SFCs sfc_library;

        visualization_msgs::MarkerArray convertLSCsToMarkerArrayMsg(
                const std::vector<Obstacle>& obstacles,
                const std::vector<std_msgs::ColorRGBA>& colors,
                double agent_radius) const;

        visualization_msgs::MarkerArray convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA& color,
                                                                    double agent_radius) const;

        SFC expandSFCFromPoint(const point_t& point, double agent_radius);
        SFC expandSFCFromLine(const Line& line, double agent_radius);
        std::vector<double> initializeBoxFromPoints(const points_t& points) const;
        bool isObstacleInSFC(const SFC& initial_sfc, double margin);
        bool isSFCInBoundary(const SFC& sfc, double margin);
        SFC expandSFC(const SFC& initial_sfc, double margin);
    };
}


#endif //LSC_PLANNER_COLLISION_CONSTRAINTS_HPP
