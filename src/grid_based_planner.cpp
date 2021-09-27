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
                                       : distmap_obj(_distmap_obj), mission(_mission), param(_param) {}

    path_t GridBasedPlanner::plan(const octomap::point3d& current_position,
                                  const octomap::point3d& goal_position,
                                  int agent_id,
                                  double agent_radius,
                                  double agent_downwash,
                                  const std::vector<dynamic_msgs::Obstacle>& obstacles,
                                  const std::set<int>& high_priority_obstacle_ids)
    {
        updateGridInfo(current_position, agent_radius);
        updateGridMap(current_position, obstacles, agent_radius, agent_downwash, high_priority_obstacle_ids);
        updateGridMission(current_position, goal_position, agent_id);

        plan_result.grid_path = plan_impl(grid_map, grid_mission);
        plan_result.path = gridPathToPath(plan_result.grid_path);
        return plan_result.path;
    }

    void GridBasedPlanner::updateGridInfo(const octomap::point3d& current_position, double agent_radius){
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

    void GridBasedPlanner::updateGridMap(const octomap::point3d& current_position,
                                         const std::vector<dynamic_msgs::Obstacle>& obstacles,
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
        if(distmap_obj != nullptr) {
            float grid_margin = param.grid_margin;
            for (int i = 0; i < grid_info.dim[0]; i++) {
                for (int j = 0; j < grid_info.dim[1]; j++) {
                    for (int k = 0; k < grid_info.dim[2]; k++) {
                        octomap::point3d point = gridVectorToPoint3D(GridVector(i, j, k));
                        float dist = distmap_obj->getDistance(point);
                        if (dist < agent_radius + grid_margin) {
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
            obs_i = (int) round((obstacles[oi].pose.position.x - grid_info.grid_min[0] + SP_EPSILON) / grid_resolution);
            obs_j = (int) round((obstacles[oi].pose.position.y - grid_info.grid_min[1] + SP_EPSILON) / grid_resolution);
            if(param.world_dimension != 2){
                obs_k = (int) round((obstacles[oi].pose.position.z - grid_info.grid_min[2] + SP_EPSILON) / grid_resolution);
            }

            if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
                // Update static obstacle to gridmap
                int size_x, size_y, size_z = 0;
                size_x = ceil((agent_radius + obstacles[oi].dimensions[0]) / grid_resolution);
                size_y = ceil((agent_radius + obstacles[oi].dimensions[1]) / grid_resolution);
                if(param.world_dimension != 2){
                    size_z = ceil((agent_radius + obstacles[oi].dimensions[2]) / grid_resolution);
                }

                octomap::point3d current_point, closest_point_obs;
                for (int i = std::max(obs_i - size_x, 0); i <= std::min(obs_i + size_x, grid_info.dim[0] - 1); i++) {
                    for (int j = std::max(obs_j - size_y, 0); j <= std::min(obs_j + size_y, grid_info.dim[1] - 1); j++) {
                        for (int k = std::max(obs_k - size_z, 0); k <= std::min(obs_k + size_z, grid_info.dim[2] - 1); k++) {
                            current_point = gridVectorToPoint3D(GridVector(i,j,k), param.world_dimension);
                            ClosestPoints closest_points;
                            closest_points = closestPointsBetweenPointAndStaticObs(current_point, obstacles[oi],
                                                                                   param.world_dimension,
                                                                                   param.world_z_2d);
                            closest_point_obs = closest_points.closest_point2;
                            double dist = (closest_point_obs - current_point).norm();
                            if (dist < agent_radius + SP_EPSILON_FLOAT) {
                                grid_map.grid[i][j][k] = GP_OCCUPIED;
                            }
                        }
                    }
                }
            }
            else if(obstacles[oi].type == ObstacleType::AGENT){
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
                            octomap::point3d point = gridVectorToPoint3D(GridVector(i, j, k));
                            dist = sqrt(pow(point.x() - obstacles[oi].pose.position.x, 2) +
                                        pow(point.y() - obstacles[oi].pose.position.y, 2) +
                                        pow((point.z() - obstacles[oi].pose.position.z) / downwash_total, 2));
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

    void GridBasedPlanner::updateGridMission(const octomap::point3d& current_position,
                                             const octomap::point3d& goal_position,
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

    path_t GridBasedPlanner::gridPathToPath(const gridpath_t& grid_path) const{
        path_t path;
        for(auto& grid_point : grid_path){
            path.emplace_back(gridVectorToPoint3D(grid_point));
        }
        return path;
    }

    octomap::point3d GridBasedPlanner::gridVectorToPoint3D(const GridVector& grid_vector) const{
        double grid_resolution = param.grid_resolution;
        return octomap::point3d(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                                grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                                grid_info.grid_min[2] + grid_vector[2] * grid_resolution);
    }

    octomap::point3d GridBasedPlanner::gridVectorToPoint3D(const GridVector& grid_vector, int dimension) const{
        double grid_resolution = param.grid_resolution;
        octomap::point3d point;
        if(dimension == 2) {
            point = octomap::point3d(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                                     grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                                     param.world_z_2d);
        }
        else {
            point = octomap::point3d(grid_info.grid_min[0] + grid_vector[0] * grid_resolution,
                                     grid_info.grid_min[1] + grid_vector[1] * grid_resolution,
                                     grid_info.grid_min[2] + grid_vector[2] * grid_resolution);
        }

        return point;
    }

    GridVector GridBasedPlanner::point3DToGridVector(const octomap::point3d& point) const{
        double grid_resolution = param.grid_resolution;
        return GridVector((int) round((point.x() - grid_info.grid_min[0]) / grid_resolution),
                          (int) round((point.y() - grid_info.grid_min[1]) / grid_resolution),
                          (int) round((point.z() - grid_info.grid_min[2]) / grid_resolution));
    }

    std::vector<octomap::point3d> GridBasedPlanner::getFreeGridPoints(){
        std::vector<octomap::point3d> free_grid_points;
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

    octomap::point3d GridBasedPlanner::findLOSFreeGoal(const octomap::point3d& current_position,
                                                       const octomap::point3d& goal_position,
                                                       const std::vector<dynamic_msgs::Obstacle>& obstacles,
                                                       double agent_radius,
                                                       const std::vector<octomap::point3d>& additional_check_positions) {
        octomap::point3d los_free_goal = current_position;

        path_t path = plan_result.path;
        path.emplace_back(goal_position);

        std::vector<octomap::point3d> start_positions = additional_check_positions;
        start_positions.emplace_back(current_position);

        for (int i = 0; i < 6; i++) {
            double margin_ratio = 1.5 - 0.1 * i;
            for (const auto &point : path) {
                bool is_safe = true;
                for (const auto &start_position: start_positions) {
                    for (const auto &obstacle : obstacles) {
                        if (obstacle.type == ObstacleType::STATICOBSTACLE) {
                            if (checkCollisionBetweenLineSegmentAndBox(obstacle,
                                                                       start_position, point,
                                                                       agent_radius, param.world_dimension)) {
                                is_safe = false;
                                break;
                            }
                        }
                    }

                    if (is_safe and distmap_obj != nullptr) {
                        is_safe = castRay(start_position, point, agent_radius * margin_ratio);
                    }

                    if (not is_safe) {
                        break;
                    }
                }

                if (is_safe) {
                    los_free_goal = point;
                }
                else {
                    break;
                }
            }

            if((los_free_goal - current_position).norm() > 0.3){
                break;
            }
        }

        octomap::point3d delta = los_free_goal - current_position;
        if(delta.norm() > param.goal_radius){
            los_free_goal = current_position + delta.normalized() * param.goal_radius;
        }

        return los_free_goal;
    }

    bool GridBasedPlanner::castRay(const octomap::point3d& current_position,
                                   const octomap::point3d& goal_position,
                                   double agent_radius)
    {
        double safe_dist_curr, safe_dist_goal, dist_to_goal, dist_threshold;
        double max_dist = 1.0; //TODO: parameterization
        dist_to_goal = (current_position - goal_position).norm();
        dist_threshold = sqrt(0.25 * dist_to_goal * dist_to_goal + agent_radius * agent_radius);
        safe_dist_curr = distmap_obj->getDistance(current_position);
        safe_dist_goal = distmap_obj->getDistance(goal_position);

        if(safe_dist_curr < agent_radius + 0.5 * param.world_resolution - SP_EPSILON_FLOAT){
            return false;
        }
        if(safe_dist_goal < agent_radius + 0.5 * param.world_resolution - SP_EPSILON_FLOAT){
            return false;
        }
        if(dist_threshold < max_dist and safe_dist_curr > dist_threshold and safe_dist_goal > dist_threshold){
            return true;
        }

        octomap::point3d mid_position = (current_position + goal_position) * 0.5;
        return castRay(current_position, mid_position, agent_radius)
               && castRay(mid_position, goal_position, agent_radius);
    }
}