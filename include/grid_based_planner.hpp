#ifndef LSC_PLANNER_GRIDBASEDPLANNER_HPP
#define LSC_PLANNER_GRIDBASEDPLANNER_HPP

#include <Astar-3D/astarplanner.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <mission.hpp>
#include <param.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <utility>

#define GP_OCCUPIED 1
#define GP_EMPTY 0
#define GP_INFINITY 10000

// Wrapper for grid based path planner
namespace DynamicPlanning {
    struct GridVector{
    public:
        GridVector(){
            data = {-1, -1, -1};
        }
        GridVector(int i, int j, int k){
            data = {i, j, k};
        }


        GridVector operator+(const GridVector& other_node) const;
        GridVector operator-(const GridVector& other_node) const;
        GridVector operator*(int integer) const;
        GridVector operator-() const;
        bool operator== (const GridVector &other_node) const;
        bool operator< (const GridVector &other_node) const;
        [[nodiscard]] int dot(const GridVector& other_agent) const;
        [[nodiscard]] double norm() const;

        [[nodiscard]] int i() const { return data[0]; }
        [[nodiscard]] int j() const { return data[1]; }
        [[nodiscard]] int k() const { return data[2]; }
        int& operator[](unsigned int idx) { return data[idx]; }
        const int& operator[](unsigned int idx) const { return data[idx]; }

        [[nodiscard]] std::array<int,3> toArray() const { return data; }

    private:
        std::array<int,3> data{-1,-1,-1};
    };

    struct GridInfo{
        std::array<double,3> grid_min;
        std::array<double,3> grid_max;
        std::array<int,3> dim;
    };

    struct GridMap{
        std::vector<std::vector<std::vector<int>>> grid;

        int getValue(GridVector grid_node) const{
            return grid[grid_node[0]][grid_node[1]][grid_node[2]];
        }

        void setValue(GridVector grid_node, int value){
            grid[grid_node[0]][grid_node[1]][grid_node[2]] = value;
        }
    };

    struct GridMission{
        GridVector start_point;
        GridVector goal_point;
    };

    typedef std::vector<GridVector> gridpath_t;

    struct PlanResult{
        points_t path;
        gridpath_t grid_path;
    };

    class GridBasedPlanner {
    public:
        GridBasedPlanner(const std::shared_ptr<DynamicEDTOctomap>& _distmap_obj,
                         const DynamicPlanning::Mission &_mission,
                         const DynamicPlanning::Param &_param);

        points_t plan(const point_t& current_position,
                      const point_t& goal_position,
                      int agent_id,
                      double agent_radius,
                      double agent_downwash,
                      const std::vector<Obstacle>& obstacles = {},
                      const std::set<int>& high_priority_obstacle_ids = {});

        // Getter
        points_t getFreeGridPoints();

        // Goal
        point_t findLOSFreeGoal(const point_t& current_position,
                                const point_t& goal_position,
                                const std::vector<Obstacle>& obstacles,
                                double agent_radius);

    private:
        Mission mission;
        Param param;
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;

        GridInfo grid_info{};
        GridMap grid_map;
        GridMission grid_mission;

        PlanResult plan_result;

        void updateGridInfo(const point_t& current_position, double agent_radius);

        void updateGridMap(const point_t& current_position,
                           const std::vector<Obstacle>& obstacles,
                           double agent_radius,
                           double agent_downwash,
                           const std::set<int>& high_priority_obstacle_ids = {});

        void updateGridMission(const point_t& current_position,
                               const point_t& goal_position,
                               int agent_id);

        bool isValid(const GridVector& grid_node);

        bool isOccupied(const GridMap& map, const GridVector& grid_node);

        static gridpath_t plan_impl(const GridMap& grid_map, const GridMission& grid_mission);

        static gridpath_t planAstar(const GridMap& grid_map, const GridMission& grid_mission);

        [[nodiscard]] points_t gridPathToPath(const gridpath_t& grid_path) const;

        [[nodiscard]]point_t gridVectorToPoint3D(const GridVector& grid_vector) const;

        [[nodiscard]] point_t gridVectorToPoint3D(const GridVector& grid_vector, int dimension) const;

        [[nodiscard]] GridVector point3DToGridVector(const point_t& point) const;

        bool castRay(const point_t& current_position, const point_t& goal_position, double agent_radius);
    };
}

#endif //LSC_PLANNER_GRIDBASEDPLANNER_HPP
