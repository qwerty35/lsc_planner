#include <grid_based_planner.hpp>

namespace DynamicPlanning {
    GridVector GridVector::operator+(const GridVector& other_node) const {
        return {i() + other_node.i(), j() + other_node.j(), k() + other_node.k()};
    }

    GridVector GridVector::operator-(const GridVector &other_node) const {
        return {i() - other_node.i(), j() - other_node.j(), k() - other_node.k()};
    }

    GridVector GridVector::operator-() const {
        return {-i(), -j(), -k()};
    }

    GridVector GridVector::operator*(int integer) const {
        return {i() * integer, j() * integer, k() * integer};
    }

    bool GridVector::operator== (const GridVector &other_node) const {
        for (unsigned int i=0; i<3; i++) {
            if (operator[](i) != other_node[i])
                return false;
        }
        return true;
    }

    bool GridVector::operator< (const GridVector &other_node) const {
        for (unsigned int i=0; i<3; i++) {
            if (operator[](i) < other_node[i]){
                return true;
            }
            else if(operator[](i) > other_node[i]){
                return false;
            }
        }
        return false;
    }

    int GridVector::dot(const GridVector& other_agent) const {
        return i() * other_agent.i() + j() * other_agent.j() + k() * other_agent.k();
    }

    double GridVector::norm() const {
        return sqrt(i() * i() + j() * j() + k() * k());
    }

    GridBasedPlanner::GridBasedPlanner(const std::shared_ptr<DynamicEDTOctomap>& _distmap_obj,
                                       const DynamicPlanning::Mission &_mission,
                                       const DynamicPlanning::Param &_param)
                                       : distmap_ptr(_distmap_obj), mission(_mission), param(_param) {}

    points_t GridBasedPlanner::plan(const point_t& current_position, const point_t& goal_position,
                                    int agent_id, double agent_radius, double agent_downwash,
                                    const std::vector<Obstacle>& obstacles,
                                    const std::set<int>& high_priority_obstacle_ids)
    {
        updateGridInfo(current_position, agent_radius);
        updateGridMap(current_position, obstacles, agent_radius, agent_downwash, high_priority_obstacle_ids);
        updateGridMission(current_position, goal_position, agent_id);

        plan_result.grid_path = plan_impl(grid_map, grid_mission);
        plan_result.path = gridPathToPath(plan_result.grid_path);
        return plan_result.path;
    }

    void GridBasedPlanner::updateGridInfo(const point_t& current_position, double agent_radius){
        double grid_resolution = param.grid_resolution;
        for(int i = 0; i < 3; i++) {
//            grid_info.grid_min[i] = current_position(i) -
//                              floor((current_position(i) - mission.world_min(i) + SP_EPSILON) / grid_resolution) *
//                              grid_resolution;
//            grid_info.grid_max[i] = current_position(i) +
//                              floor((mission.world_max(i) - current_position(i) + SP_EPSILON) / grid_resolution) *
//                              grid_resolution;
            grid_info.grid_min[i] = -floor((- mission.world_min(i) + SP_EPSILON) / grid_resolution) * grid_resolution;
            grid_info.grid_max[i] = floor((mission.world_max(i) + SP_EPSILON) / grid_resolution) * grid_resolution;
        }
        if(param.world_dimension == 2){
            grid_info.grid_min[2] = param.world_z_2d;
            grid_info.grid_max[2] = param.world_z_2d;
        }

        for (int i = 0; i < 3; i++) {
            grid_info.dim[i] = (int) round((grid_info.grid_max[i] - grid_info.grid_min[i]) / grid_resolution) + 1;
        }
    }

    void GridBasedPlanner::updateGridMap(const point_t& current_position,
                                         const std::vector<Obstacle>& obstacles,
                                         double agent_radius,
                                         double agent_downwash,
                                         const std::set<int>& high_priority_obstacle_ids) {
        // Initialize gridmap
        grid_map.grid.resize(grid_info.dim[0]);
        for (int i = 0; i < grid_info.dim[0]; i++) {
            grid_map.grid[i].resize(grid_info.dim[1]);
            for (int j = 0; j < grid_info.dim[1]; j++) {
                grid_map.grid[i][j].resize(grid_info.dim[2]);
                for (int k = 0; k < grid_info.dim[2]; k++) {
                    grid_map.grid[i][j][k] = GP_EMPTY;
                }
            }
        }

        // Update distmap to gridmap
        if(distmap_ptr != nullptr) {

            for (int i = 0; i < grid_info.dim[0]; i++) {
                for (int j = 0; j < grid_info.dim[1]; j++) {
                    for (int k = 0; k < grid_info.dim[2]; k++) {
                        float dist;
                        point_t search_point, closest_point;
                        search_point = gridVectorToPoint3D(GridVector(i, j, k));
                        distmap_ptr->getDistanceAndClosestObstacle(search_point, dist, closest_point);

                        // Due to numerical error of getDistance function, explicitly compute distance to obstacle
                        double dist_to_obs = search_point.distance(closest_point);

                        if (dist_to_obs < agent_radius + param.grid_margin) {
                            grid_map.grid[i][j][k] = GP_OCCUPIED;
                        }
                    }
                }
            }
        }

        int obs_i, obs_j, obs_k = 0;
        int N_obs = obstacles.size();
        double grid_resolution = param.grid_resolution;
        for (int oi = 0; oi < N_obs; oi++) {
            obs_i = (int) round((obstacles[oi].position.x() - grid_info.grid_min[0] + SP_EPSILON) / grid_resolution);
            obs_j = (int) round((obstacles[oi].position.y() - grid_info.grid_min[1] + SP_EPSILON) / grid_resolution);
            if(param.world_dimension != 2){
                obs_k = (int) round((obstacles[oi].position.z() - grid_info.grid_min[2] + SP_EPSILON) / grid_resolution);
            }

            if(obstacles[oi].type == ObstacleType::AGENT){
                // Update higher priority agent as an obstacle to gridmap
                if(not isElementInSet(high_priority_obstacle_ids, (int)obstacles[oi].id)){
                    continue;
                }

                int size_xy, size_z;
                size_xy = ceil((agent_radius + obstacles[oi].radius) / grid_resolution);
                size_z = ceil((agent_radius * agent_downwash + obstacles[oi].radius * obstacles[oi].downwash) /
                                  grid_resolution);

                double downwash_total, dist;
                downwash_total = (agent_radius * agent_downwash + obstacles[oi].radius * obstacles[oi].downwash) /
                                 (agent_radius + obstacles[oi].radius);

                for (int i = std::max(obs_i - size_xy, 0); i <= std::min(obs_i + size_xy, grid_info.dim[0] - 1); i++) {
                    for (int j = std::max(obs_j - size_xy, 0); j <= std::min(obs_j + size_xy, grid_info.dim[1] - 1); j++) {
                        for (int k = std::max(obs_k - size_z, 0); k <= std::min(obs_k + size_z, grid_info.dim[2] - 1); k++) {
                            point_t point = gridVectorToPoint3D(GridVector(i, j, k));

                            dist = ellipsoidalDistance(point, obstacles[oi].position, downwash_total);
                            if (dist < agent_radius + obstacles[oi].radius) {
                                grid_map.grid[i][j][k] = GP_OCCUPIED;
                            }
                        }
                    }
                }
            }
            else if(obstacles[oi].type == ObstacleType::DYNAMICOBSTACLE){
                //TODO: dynamic obstacle case
            }
        }
    }

    void GridBasedPlanner::updateGridMission(const point_t& current_position,
                                             const point_t& goal_position,
                                             int agent_id)
    {
        grid_mission.start_point = point3DToGridVector(current_position);
        grid_mission.goal_point = point3DToGridVector(goal_position);

        if(param.world_dimension == 2) {
            grid_mission.start_point[2] = 0;
            grid_mission.goal_point[2] = 0;
        }

        if (grid_map.getValue(grid_mission.start_point) == GP_OCCUPIED) {
//            ROS_WARN_STREAM("[GridBasedPlanner] Start point of agent " << std::to_string(agent_id) << " is occluded");
            int min_dist = SP_INFINITY;
            GridVector closest_point = grid_mission.start_point;
            for(int i = -2; i < 3; i++){
                for(int j = -2; j < 3; j++) {
                    for (int k = 2 - param.world_dimension; k < param.world_dimension - 1; k++) {
                        GridVector candidate_point = grid_mission.start_point + GridVector(i, j, k);
                        if (not isOccupied(grid_map, candidate_point)) {
                            int dist = abs(i) + abs(j) + abs(k);
                            if (dist < min_dist) {
                                min_dist = dist;
                                closest_point = candidate_point;
                            }
                        }
                    }
                }
            }
            grid_mission.start_point = closest_point;

            if(grid_map.getValue(grid_mission.start_point) == GP_OCCUPIED){
//                ROS_WARN_STREAM("[GridBasedPlanner] Start point of agent " << std::to_string(agent_id) << " is occluded again");
                grid_map.setValue(grid_mission.start_point, GP_EMPTY);
            }
        }

        // simple heuristic
//        if (grid_map.getValue(grid_mission.start_point) == GP_OCCUPIED) {
//            ROS_WARN_STREAM("[GridBasedPlanner] Start point of agent " << agent.id <<" is occluded");
//            grid_map.setValue(grid_mission.start_point, GP_EMPTY);
//        }

//        if (map.getValue(grid_mission.goal_point) == 1) {
//            ROS_ERROR("[GridBasedPlanner] Goal point is occluded");
//            throw PlanningReport::INITTRAJGENERATIONFAILED;
//        }
    }

    bool GridBasedPlanner::isValid(const GridVector& grid_node){
        for(int i = 0; i < 3; i++){
            if(grid_node[i] < 0 or grid_node[i] >= grid_info.dim[i]){
                return false;
            }
        }

        return true;
    }

    bool GridBasedPlanner::isOccupied(const GridMap& map, const GridVector& grid_node){
        for(int i = 0; i < 3; i++){
            if(grid_node[i] < 0 || grid_node[i] > grid_info.dim[i] - 1){
                return true;
            }
        }
        return map.getValue(grid_node) == GP_OCCUPIED;
    }

    gridpath_t GridBasedPlanner::plan_impl(const GridMap& grid_map, const GridMission& grid_mission){
        gridpath_t grid_path;
//        if(mode_selection){
        grid_path = planAstar(grid_map, grid_mission);
//        }
//        else {
//            throw std::invalid_argument("[GridBasedPlanner] Invalid grid based planner mode");
//        }

        return grid_path;
    }

    gridpath_t GridBasedPlanner::planAstar(const GridMap& map, const GridMission& grid_mission) {
        AstarPlanner planner;
        EnvironmentOptions default_options;
        SearchResult sr = planner.plan(map.grid,
                                       grid_mission.start_point.toArray(),
                                       grid_mission.goal_point.toArray(),
                                       default_options);

        gridpath_t grid_path;
        if(sr.pathfound){
            for(auto& node : sr.lppath->List){
                GridVector grid_node(node.i, node.j, node.z);
                grid_path.emplace_back(grid_node);
            }
        }

        return grid_path;
    }

    points_t GridBasedPlanner::gridPathToPath(const gridpath_t& grid_path) const{
        points_t path;
        for(auto& grid_point : grid_path){
            path.emplace_back(gridVectorToPoint3D(grid_point));
        }
        return path;
    }

    point_t GridBasedPlanner::gridVectorToPoint3D(const GridVector& grid_vector) const{
        double grid_resolution = param.grid_resolution;
        return point_t(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                       grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                       grid_info.grid_min[2] + grid_vector[2] * grid_resolution);
    }

    point_t GridBasedPlanner::gridVectorToPoint3D(const GridVector& grid_vector, int dimension) const{
        double grid_resolution = param.grid_resolution;
        point_t point;
        if(dimension == 2) {
            point = point_t(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                                     grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                                     param.world_z_2d);
        }
        else {
            point = point_t(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                                     grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                                     grid_info.grid_min[2] + grid_vector[2] * grid_resolution);
        }

        return point;
    }

    GridVector GridBasedPlanner::point3DToGridVector(const point_t& point) const{
        double grid_resolution = param.grid_resolution;
        return GridVector((int) round((point.x() - grid_info.grid_min[0]) / grid_resolution),
                          (int) round((point.y() - grid_info.grid_min[1]) / grid_resolution),
                          (int) round((point.z() - grid_info.grid_min[2]) / grid_resolution));
    }

    points_t GridBasedPlanner::getFreeGridPoints(){
        points_t free_grid_points;
        for (int i = 0; i < grid_info.dim[0]; i++) {
            for (int j = 0; j < grid_info.dim[1]; j++) {
                for (int k = 0; k < grid_info.dim[2]; k++) {
                    if(grid_map.grid[i][j][k] == GP_EMPTY){
                        free_grid_points.emplace_back(gridVectorToPoint3D(GridVector(i, j, k)));
                    }
                }
            }
        }
        return free_grid_points;
    }

    point_t GridBasedPlanner::findLOSFreeGoal(const point_t& current_position, const point_t& goal_position,
                                              const std::vector<Obstacle>& obstacles, double agent_radius) {
        point_t los_free_goal = current_position;

        points_t path = plan_result.path;
        path.emplace_back(goal_position);

        if (distmap_ptr != nullptr) {
            for (const auto &point : path) {
                bool is_safe = castRay(current_position, point, agent_radius);

                if (is_safe) {
                    los_free_goal = point;
                }
                else {
                    break;
                }
            }
        }

        if(los_free_goal.distance(current_position) < SP_EPSILON_FLOAT and path.size() > 2){
            los_free_goal = path[1];
        }

//        point_t delta = los_free_goal - current_position;
//        if(delta.norm() > param.goal_radius){
//            los_free_goal = current_position + delta.normalized() * param.goal_radius;
//        }

        return los_free_goal;
    }

    bool GridBasedPlanner::castRay(const point_t& current_position,
                                   const point_t& goal_position,
                                   double agent_radius)
    {
        double safe_dist_curr, safe_dist_goal, dist_to_goal, dist_threshold;
        double max_dist = 1.0; //TODO: parameterization
        dist_to_goal = (current_position - goal_position).norm();
        dist_threshold = sqrt(0.25 * dist_to_goal * dist_to_goal + agent_radius * agent_radius);

//        safe_dist_curr = distmap_ptr->getDistance(current_position);
//        safe_dist_goal = distmap_ptr->getDistance(goal_position);

        float dist;
        point_t closest_point;
        distmap_ptr->getDistanceAndClosestObstacle(current_position, dist, closest_point);
        safe_dist_curr = current_position.distance(closest_point);

        distmap_ptr->getDistanceAndClosestObstacle(goal_position, dist, closest_point);
        safe_dist_goal = goal_position.distance(closest_point);

        if(safe_dist_curr < agent_radius + 0.5 * param.world_resolution - SP_EPSILON_FLOAT){
            return false;
        }
        if(safe_dist_goal < agent_radius + 0.5 * param.world_resolution - SP_EPSILON_FLOAT){
            return false;
        }
        if(dist_threshold < max_dist and safe_dist_curr > dist_threshold and safe_dist_goal > dist_threshold){
            return true;
        }

        point_t mid_position = (current_position + goal_position) * 0.5;
        return castRay(current_position, mid_position, agent_radius)
               && castRay(mid_position, goal_position, agent_radius);
    }
}