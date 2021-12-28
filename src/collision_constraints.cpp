#include <collision_constraints.hpp>

namespace DynamicPlanning{
    LSC::LSC(const point3d& _obs_control_point,
             const point3d& _normal_vector,
             double _d)
            : obs_control_point(_obs_control_point), normal_vector(_normal_vector), d(_d) {}

    visualization_msgs::Marker LSC::convertToMarker(double agent_radius, const std::string& world_frame_id) const{
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id;
        msg_marker.type = visualization_msgs::Marker::CUBE;
        msg_marker.action = visualization_msgs::Marker::ADD;

        double box_scale = 40; //TODO: parameterization
        msg_marker.scale.x = box_scale;
        msg_marker.scale.y = box_scale;
        msg_marker.scale.z = box_scale;

        double distance = -(d - agent_radius) + box_scale / 2;
        Eigen::Vector3d V3d_normal_vector(normal_vector.x(), normal_vector.y(), normal_vector.z());
        Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, -V3d_normal_vector);

        msg_marker.pose.position = point3DToPointMsg(obs_control_point - normal_vector * distance);
        msg_marker.pose.orientation = quaternionToQuaternionMsg(q);

        return msg_marker;
    }


    SFC::SFC(const point3d& _box_min, const point3d& _box_max){
        box_min = _box_min;
        box_max = _box_max;
    }

    LSCs SFC::convertToLSCs(int dim) const{
        point3d normal_vector_min, normal_vector_max;
        point3d zero_point = point3d(0, 0, 0);
        double d_min, d_max;

        std::vector<LSC> lscs;
        lscs.resize(2 * dim);
        for(int i = 0; i < dim; i++) {
            normal_vector_min = zero_point;
            normal_vector_max = zero_point;
            normal_vector_min(i) = 1;
            normal_vector_max(i) = -1;
            d_min = box_min(i);
            d_max = -box_max(i);

            LSC lsc_min(zero_point, normal_vector_min, d_min);
            LSC lsc_max(zero_point, normal_vector_max, d_max);
            lscs[2 * i] = lsc_min;
            lscs[2 * i + 1] = lsc_max;
        }

        return lscs;
    }

    visualization_msgs::Marker SFC::convertToMarker(double agent_radius, const std::string& world_frame_id) const{
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id; //TODO: frame id
        msg_marker.type = visualization_msgs::Marker::LINE_LIST;
        msg_marker.action = visualization_msgs::Marker::ADD;
        msg_marker.pose.position = defaultPoint();
        msg_marker.pose.orientation = defaultQuaternion();
        msg_marker.scale.x = 0.03;

        point3d inflation_vector(agent_radius, agent_radius, agent_radius);
        SFC inflated_box = SFC(box_min - inflation_vector, box_max + inflation_vector);
        lines_t edges = inflated_box.getEdges();
        for(const auto& edge : edges){
            msg_marker.points.emplace_back(point3DToPointMsg(edge.start_point));
            msg_marker.points.emplace_back(point3DToPointMsg(edge.end_point));
        }

        return msg_marker;
    }

    bool SFC::isPointInSFC(const point3d& point) const{
        return point.x() > box_min.x() - SP_EPSILON_FLOAT &&
               point.y() > box_min.y() - SP_EPSILON_FLOAT &&
               point.z() > box_min.z() - SP_EPSILON_FLOAT &&
               point.x() < box_max.x() + SP_EPSILON_FLOAT &&
               point.y() < box_max.y() + SP_EPSILON_FLOAT &&
               point.z() < box_max.z() + SP_EPSILON_FLOAT;
    }

    bool SFC::isLineInSFC(const Line& line) const{
        return isPointInSFC(line.start_point) && isPointInSFC(line.end_point);
    }

    bool SFC::isSFCInBoundary(const point3d& world_min, const point3d& world_max,
                              double margin) const {
        return box_min.x() > world_min.x() + margin - SP_EPSILON &&
               box_min.y() > world_min.y() + margin - SP_EPSILON &&
               box_min.z() > world_min.z() + margin - SP_EPSILON &&
               box_max.x() < world_max.x() - margin + SP_EPSILON &&
               box_max.y() < world_max.y() - margin + SP_EPSILON &&
               box_max.z() < world_max.z() - margin + SP_EPSILON;
    }

    bool SFC::isSuperSetOfConvexHull(const points_t& convex_hull) const{
        float min_value, max_value;
        for(int i = 0; i < 3; i++){
            std::vector<float> points_i;
            for(auto point : convex_hull){
                points_i.emplace_back(point(i));
            }
            min_value = *std::min_element(points_i.begin(), points_i.end());
            max_value = *std::max_element(points_i.begin(), points_i.end());
            if(min_value < box_min(i) - SP_EPSILON_FLOAT || max_value > box_max(i) + SP_EPSILON_FLOAT){
                return false;
            }
        }

        return true;
    }

    bool SFC::intersectWith(const SFC& other_sfc) const{
        SFC inter_sfc = intersection(other_sfc);
        for(int i = 0; i <3; i++){
            if(inter_sfc.box_min(i) > inter_sfc.box_max(i) - SP_EPSILON_FLOAT){
                return false;
            }
        }
        return true;
    }

    SFC SFC::unify(const SFC& other_sfc) const{
        SFC unified_box;
        for(int i = 0; i < 3; i++){
            unified_box.box_min(i) = std::min(box_min(i), other_sfc.box_min(i));
            unified_box.box_max(i) = std::max(box_max(i), other_sfc.box_max(i));
        }
        return unified_box;
    }

    SFC SFC::intersection(const SFC& other_sfc) const{
        SFC inter_box;
        for(int i = 0; i < 3; i++){
            inter_box.box_min(i) = std::max(box_min(i), other_sfc.box_min(i));
            inter_box.box_max(i) = std::min(box_max(i), other_sfc.box_max(i));
        }
        return inter_box;
    }

    double SFC::distanceToPoint(const point3d& point) const{
        if(isPointInSFC(point)){
            return 0;
        }

        point3d closest_point = point;
        for(int i = 0; i < 3; i++){
            if(point(i) < box_min(i)){
                closest_point(i) = box_min(i);
            }
            else if(point(i) > box_max(i)){
                closest_point(i) = box_max(i);
            }
        }

        return (point - closest_point).norm();
    }

    double SFC::distanceToInnerPoint(const point3d& point) const{
        if(not isPointInSFC(point)){
            return -1;
        }

        double dist, min_dist = SP_INFINITY;
        for(int i = 0; i < 3; i++){
            dist = abs(point(i) - box_min(i));
            if(dist < min_dist){
                min_dist = dist;
            }

            dist = abs(point(i)) - box_max(i);
            if(dist < min_dist){
                min_dist = dist;
            }
        }

        return min_dist;
    }

    double SFC::raycastFromInnerPoint(const point3d& inner_point, const point3d& direction) const{
        point3d surface_direction;
        return raycastFromInnerPoint(inner_point, direction, surface_direction);
    }

    double SFC::raycastFromInnerPoint(const point3d& inner_point, const point3d& direction, point3d& surface_direction) const{
        if(not isPointInSFC(inner_point)){
            return -1;
        }

        double a, b, k, min_dist = SP_INFINITY;
        for(int i = 0; i < 3; i++){
            point3d n_surface;
            n_surface(i) = 1;
            a = (box_min - inner_point).dot(n_surface);
            b = direction.dot(n_surface);
            if(abs(b) < SP_EPSILON_FLOAT) {
                continue;
            }

            k = a / b;
            if(k > 0 and k < min_dist){
                min_dist = k;
                surface_direction = n_surface;
            }
        }

        for(int i = 0; i < 3; i++){
            point3d n_surface;
            n_surface(i) = -1;
            a = (box_max - inner_point).dot(n_surface);
            b = direction.dot(n_surface);
            if(abs(b) < SP_EPSILON_FLOAT) {
                continue;
            }

            k = a / b;
            if(k >= 0 and k < min_dist){
                min_dist = k;
                surface_direction = n_surface;
            }
        }

        return min_dist;
    }

    points_t SFC::getVertices() const{
        points_t vertices;
        vertices.emplace_back(box_min);
        vertices.emplace_back(box_max);
        for(int i = 0; i < 3; i++){
            point3d point1 = box_min;
            point3d point2 = box_max;
            point1(i) = box_max(i);
            point2(i) = box_min(i);
            vertices.emplace_back(point1);
            vertices.emplace_back(point2);
        }

        return vertices;
    }

    lines_t SFC::getEdges() const{
        lines_t edges;

        // find edges
        point3d vertex1, vertex2, vertex3;

        vertex1 = box_min;
        for(int i = 0; i < 3; i++){
            vertex2 = box_min;
            vertex2(i) = box_max(i);
            edges.emplace_back(Line(vertex1, vertex2));

            for(int j = 0; j < 3; j++){
                if(i == j) continue;
                vertex3 = vertex2;
                vertex3(j) = box_max(j);
                edges.emplace_back(Line(vertex2, vertex3));
            }
        }

        vertex2 = box_max;
        for(int i = 0; i < 3; i++){
            vertex1 = box_max;
            vertex1(i) = box_min(i);
            edges.emplace_back(Line(vertex1, vertex2));
        }

        return edges;
    }

    CollisionConstraints::CollisionConstraints(const Param& param_, const Mission& mission_) {
        param = param_;
        mission = mission_;

        M = param.M;
        n = param.n;
        dt = param.dt;

        //SFC
        sfcs.resize(M);
        sfc_library.clear();
    }

    void CollisionConstraints::initializeSFC(const point3d& agent_position, double radius) {
        SFC sfc = expandSFCFromPoint(agent_position, radius);
        for(int m = 0; m < M; m++){
            sfcs[m] = sfc;
        }
    }

    void CollisionConstraints::initializeLSC(size_t N_obs){
        lscs.clear();
        lscs.resize(N_obs);
        for(int oi = 0; oi < N_obs; oi++){
            lscs[oi].resize(M);
            for(int m = 0; m < M; m++){
                lscs[oi][m].resize(n + 1);
            }
        }
    }

    void CollisionConstraints::updateSFCLibrary(const points_t& grid_path, double agent_radius){
        // Update sfc library using discrete path
        if(not grid_path.empty()) {
            for (int i = 0; i < grid_path.size() - 1; i++) {
                Line line_curr = Line(grid_path[i], grid_path[i + 1]);
                bool update = true;
                for (const auto &sfc: sfc_library) {
                    if (sfc.isLineInSFC(line_curr)) {
                        update = false;
                        break;
                    }
                }

                if (update) {
                    sfc_library.emplace_back(expandSFCFromLine(line_curr, agent_radius));
                }
            }
        }
    }

    void CollisionConstraints::generateFeasibleSFC(const point3d& last_point, const point3d& current_goal_position,
                                                   const points_t& grid_path, double agent_radius){
        // update sfc library
        updateSFCLibrary(grid_path, agent_radius);

        // Update sfc for segments m < M-1 from previous sfc
        for(int m = 0; m < M - 1; m++){
            sfcs[m] = sfcs[m + 1];
        }

//        // Find last point is in sfc_library
        SFC sfc_update;
        double min_dist_to_goal = SP_INFINITY;
        for(const auto& sfc : sfc_library){
            Line line(last_point, current_goal_position);
            if(sfc.isLineInSFC(line)){
                sfc_update = sfc;
                min_dist_to_goal = -1;
                break;
            }
            else if(sfc.isPointInSFC(last_point)){
                double dist = sfc.distanceToPoint(current_goal_position);
                if(dist < min_dist_to_goal){
                    sfc_update = sfc;
                    min_dist_to_goal = dist;
                }
            }
        }

        if(min_dist_to_goal == SP_INFINITY){
            ROS_WARN("[CollisionConstraints] Cannot find proper SFC in sfc_library, try naive method");
            try{
                sfc_update = expandSFCFromPoint(last_point, agent_radius);
                sfc_library.emplace_back(sfc_update);
            }
            catch(...){ //TODO: define error
                ROS_WARN("[CollisionConstraints] Cannot find proper SFC, use previous one");
                sfc_update = sfcs[M - 1]; // Reuse previous one
            }
        }

        sfcs[M - 1] = sfc_update;
    }

    point3d CollisionConstraints::findLOSFreeGoal(const point3d& last_point,
                                                  const point3d& desired_goal,
                                                  const points_t& grid_path){
        point3d los_free_goal;
        if(grid_path.empty() or sfc_library.empty()){
            los_free_goal = desired_goal;
        }
        else{
            for(const auto& grid_point : grid_path){
                for(const auto& sfc : sfc_library){
                    if(!sfc.isPointInSFC(last_point) or !sfc.isPointInSFC(grid_point)){
                        continue;
                    }

                    los_free_goal = grid_point;
                }
            }
        }

        return los_free_goal;
    }

    SFC CollisionConstraints::findProperSFC(const point3d& start_point, const point3d& goal_point){
        SFC proper_sfc, sfc_cand;
        bool find_proper_sfc = false;
        double min_dist = SP_INFINITY;

        for(const auto& sfc : sfc_library){
            if(not sfc.isPointInSFC(start_point)){
                continue;
            }

            if(sfc.isPointInSFC(goal_point)){
                proper_sfc = sfc;
                find_proper_sfc = true;
                break;
            }
            else{
                double dist = sfc.distanceToPoint(goal_point);
                if(dist < min_dist){
                    sfc_cand = sfc;
                    min_dist = dist;
                }
            }
        }

        if(not find_proper_sfc){
            if(min_dist == SP_INFINITY){ // if sfc_cand not found
                if(M > 1){
                    proper_sfc = getSFC(1); // SFC will be updated to this one.
                }
                else{
                    proper_sfc = getSFC(0);
                }
            }

            proper_sfc = sfc_cand;
        }

        return proper_sfc;
    }

    LSC CollisionConstraints::getLSC(int oi, int m, int i) const{
        return lscs[oi][m][i];
    }

    SFC CollisionConstraints::getSFC(int m) const{
        return sfcs[m];
    }

    size_t CollisionConstraints::getObsSize() const{
        return lscs.size();
    }

    std::set<int> CollisionConstraints::getSlackIndices() const{
        return obs_slack_indices;
    }

    void CollisionConstraints::setDistmap(std::shared_ptr<DynamicEDTOctomap> distmap_ptr_){
        distmap_ptr = distmap_ptr_;
    }

    void CollisionConstraints::setObsSlackIndicies(const std::set<int>& obs_slack_indices_){
        obs_slack_indices = obs_slack_indices_;
    }

    void CollisionConstraints::setLSC(int oi, int m,
                                      const points_t& obs_control_points,
                                      const vector3d& normal_vector,
                                      const std::vector<double>& ds) {
        for(int i = 0; i < n + 1; i++){
            lscs[oi][m][i] = LSC(obs_control_points[i], normal_vector, ds[i]);
        }
    }

    void CollisionConstraints::setLSC(int oi, int m,
                                      const points_t& obs_control_points,
                                      const vector3d& normal_vector,
                                      double d) {
        for(int i = 0; i < n + 1; i++){
            lscs[oi][m][i] = LSC(obs_control_points[i], normal_vector, d);
        }
    }

    void CollisionConstraints::setSFC(int m, const SFC& sfc){
        sfcs[m] = sfc;
    }

    visualization_msgs::MarkerArray CollisionConstraints::convertToMarkerArrayMsg (
            const std::vector<Obstacle>& obstacles,
            const std::vector<std_msgs::ColorRGBA>& colors,
            int agent_id, double agent_radius) const
    {
        visualization_msgs::MarkerArray msg1 = convertLSCsToMarkerArrayMsg(obstacles, colors, agent_radius);
        visualization_msgs::MarkerArray msg2 = convertSFCsToMarkerArrayMsg(colors[agent_id], agent_radius);
        msg1.markers.insert(msg1.markers.end(), msg2.markers.begin(), msg2.markers.end());
        return msg1;
    }

    visualization_msgs::MarkerArray CollisionConstraints::convertLSCsToMarkerArrayMsg (
            const std::vector<Obstacle>& obstacles,
            const std::vector<std_msgs::ColorRGBA>& colors,
            double agent_radius) const
    {
        visualization_msgs::MarkerArray msg_marker_array;

        if (lscs.empty()) {
            return msg_marker_array;
        }

        size_t N_obs = obstacles.size();
        msg_marker_array.markers.clear();
        for (int oi = 0; oi < N_obs; oi++) {
            std_msgs::ColorRGBA marker_color;
            if((int)obstacles[oi].id > -1){
                marker_color = colors[obstacles[oi].id];
                marker_color.a = 0.2;
            }
            else{
                marker_color.r = 0.0;
                marker_color.g = 0.0;
                marker_color.b = 0.0;
                marker_color.a = 0.2;
            }

            for (int m = 0; m < M; m++) {
//                visualization_msgs::Marker msg_marker = lscs[oi][m][0].convertToMarker(agent_radius);
                visualization_msgs::Marker msg_marker = lscs[oi][m][0].convertToMarker(0, param.world_frame_id);
                msg_marker.id = m * N_obs + oi;
                msg_marker.ns = "LSC" + std::to_string(m);
                msg_marker.color = marker_color;

                msg_marker_array.markers.emplace_back(msg_marker);
            }
        }

        return msg_marker_array;
    }

    visualization_msgs::MarkerArray CollisionConstraints::convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA& color,
                                                                                      double agent_radius) const
    {
        visualization_msgs::MarkerArray msg_marker_array;

        if (sfcs.empty()) {
            return msg_marker_array;
        }

        msg_marker_array.markers.clear();
        for (int m = 0; m < M; m++) {
//            visualization_msgs::Marker msg_marker = sfcs[m].box.convertToMarker(agent_radius);
            visualization_msgs::Marker msg_marker = sfcs[m].convertToMarker(0, param.world_frame_id);
            msg_marker.id = m;
            msg_marker.ns = "SFC" + std::to_string(m);
            msg_marker.color = color;
            msg_marker.color.a = 1.0;
            msg_marker_array.markers.emplace_back(msg_marker);
        }

        return msg_marker_array;
    }

    dynamic_msgs::CollisionConstraint CollisionConstraints::convertToRawMsg(
            const std::vector<Obstacle>& obstacles, int planner_seq) const {
        dynamic_msgs::CollisionConstraint msg_collision_constraint;
        msg_collision_constraint.planner_seq = planner_seq;

        dynamic_msgs::Param constraint_param;
        constraint_param.M = M;
        constraint_param.n = n;
        constraint_param.dt = dt;

        if(not lscs.empty()){
            size_t N_obs = obstacles.size();
            msg_collision_constraint.rsfcs.resize(N_obs);
            for (int oi = 0; oi < N_obs; oi++) {
                constraint_param.id = obstacles[oi].id;
                msg_collision_constraint.rsfcs[oi].param = constraint_param;
                msg_collision_constraint.rsfcs[oi].obs_control_points.resize(M * (n + 1));
                msg_collision_constraint.rsfcs[oi].normal_vector.resize(M * (n + 1));
                msg_collision_constraint.rsfcs[oi].d.resize(M * (n + 1));
                for (int m = 0; m < M; m++) {
                    for(int i = 0; i < n + 1; i++) {
                        int idx = m * (n + 1) + i;
                        msg_collision_constraint.rsfcs[oi].obs_control_points[idx] =
                                point3DToPointMsg(lscs[oi][m][i].obs_control_point);
                        msg_collision_constraint.rsfcs[oi].normal_vector[idx] =
                                point3DToPointMsg(lscs[oi][m][i].normal_vector);
                        msg_collision_constraint.rsfcs[oi].d[idx] = lscs[oi][m][i].d;
                    }
                }
            }
        }

        if(not sfcs.empty()){
            msg_collision_constraint.sfcs.resize(M);
            for(int m = 0; m < M; m++){
                msg_collision_constraint.sfcs[m].param = constraint_param;
                msg_collision_constraint.sfcs[m].box_min = point3DToPointMsg(sfcs[m].box_min);
                msg_collision_constraint.sfcs[m].box_max = point3DToPointMsg(sfcs[m].box_max);
            }
        }

        return msg_collision_constraint;
    }

    SFC CollisionConstraints::expandSFCFromPoint(const point3d& point, double agent_radius) {
        // Initialize initial_box
        SFC initial_sfc;

        for(int i = 0; i < 3; i++){
            double round_point_i = round(point(i) / param.world_resolution) * param.world_resolution;
            if(abs(point(i) - round_point_i) < SP_EPSILON_FLOAT){
                initial_sfc.box_min(i) = round_point_i;
                initial_sfc.box_max(i) = round_point_i;
            }
            else{
                initial_sfc.box_min(i) = floor(point(i) / param.world_resolution) * param.world_resolution;
                initial_sfc.box_max(i) = ceil(point(i) / param.world_resolution) * param.world_resolution;
            }
        }

        SFC sfc = expandSFC(initial_sfc, agent_radius);
        return sfc;
    }

    SFC CollisionConstraints::expandSFCFromLine(const Line& line, double agent_radius) {
        // Initialize initial_box
        SFC initial_sfc;

        for(int i = 0; i < 3; i++) {
            initial_sfc.box_min(i) =
                    round(std::min(line.start_point(i), line.end_point(i)) / param.world_resolution) *
                    param.world_resolution;
            initial_sfc.box_max(i) =
                    round(std::max(line.start_point(i), line.end_point(i)) / param.world_resolution) *
                    param.world_resolution;
        }

        SFC sfc = expandSFC(initial_sfc, agent_radius);
        return sfc;
    }

    std::vector<double> CollisionConstraints::initializeBoxFromPoints(const points_t& points) const{
        std::vector<double> box;
        box.resize(6);
        double epsilon = 1e-3;

        // Find minimum box that covers all points
        for(int i = 0; i < 3; i++){
            std::vector<float> points_i;
            for(auto point : points){
                points_i.emplace_back(point(i));
            }
            box[i] = *std::min_element(points_i.begin(), points_i.end());
            box[i+3] = *std::max_element(points_i.begin(), points_i.end());
        }

        for(int i = 0; i < 3; i++){
            box[i] = floor((box[i] + epsilon) / param.world_resolution) * param.world_resolution;
            box[i+3] = ceil((box[i+3] - epsilon) / param.world_resolution) * param.world_resolution;
        }

        return box;
    }

    bool CollisionConstraints::isObstacleInSFC(const SFC& sfc, double margin) {
        std::array<int, 3> sfc_size = {0, 0, 0};
        for(int i = 0; i < 3; i++){
            sfc_size[i] = (int)round((sfc.box_max(i) - sfc.box_min(i)) / param.world_resolution) + 1;
        }

        point3d delta;
        std::array<size_t, 3> iter = {0, 0, 0};
//        for (iter[0] = 0; iter[0] < std::max(sfc_size[0], 2); iter[0]++) {
//            for (iter[1] = 0; iter[1] < std::max(sfc_size[1], 2); iter[1]++) {
//                for (iter[2] = 0; iter[2] < std::max(sfc_size[2], 2); iter[2]++) {
        for (iter[0] = 0; iter[0] < sfc_size[0]; iter[0]++) {
            for (iter[1] = 0; iter[1] < sfc_size[1]; iter[1]++) {
                for (iter[2] = 0; iter[2] < sfc_size[2]; iter[2]++) {
                    float dist;
                    point3d search_point, closest_point;
                    for(int i = 0; i < 3; i++){
                        search_point(i) = sfc.box_min(i) + iter[i] * param.world_resolution;
                    }
                    distmap_ptr->getDistanceAndClosestObstacle(search_point, dist, closest_point);

                    // Due to numerical error of getDistance function, explicitly compute distance to obstacle
                    double dist_to_obs = search_point.distance(closest_point);

                    if (dist_to_obs < margin) {
                        return true;
                    }

//                    point_t search_point;
//                    for(int i = 0; i < 3; i++){
//                        if(sfc_size[i] == 1 and iter[i] > 0){
//                            search_point(i) = sfc.box_min(i);
//                        }
//                        else{
//                            search_point(i) = sfc.box_min(i) + iter[i] * param.world_resolution;
//                        }
//                    }
//
//                    // Find delta to compensate numerical error
//                    for(int i = 0; i < 3; i++){
//                        if (iter[i] == 0 && sfc.box_min(i) > mission.world_min(i) + SP_EPSILON_FLOAT) {
//                            delta(i) = -SP_EPSILON_FLOAT;
//                        }
//                        else{
//                            delta(i) = SP_EPSILON_FLOAT;
//                        }
//                    }
//
//                    search_point = search_point + delta;
//                    float dist = distmap_ptr->getDistance(search_point);
//                    if (dist < margin + 0.5 * param.world_resolution - SP_EPSILON_FLOAT) { // Add 0.5 * resolution to avoid numerical error
//                        return true;
//                    }
                }
            }
        }

        return false;
    }

    bool CollisionConstraints::isSFCInBoundary(const SFC& sfc, double margin) {
        return sfc.box_min.x() > mission.world_min.x() + margin - SP_EPSILON &&
               sfc.box_min.y() > mission.world_min.y() + margin - SP_EPSILON &&
               sfc.box_min.z() > mission.world_min.z() + margin - SP_EPSILON &&
               sfc.box_max.x() < mission.world_max.x() - margin + SP_EPSILON &&
               sfc.box_max.y() < mission.world_max.y() - margin + SP_EPSILON &&
               sfc.box_max.z() < mission.world_max.z() - margin + SP_EPSILON;
    }

    SFC CollisionConstraints::expandSFC(const SFC& initial_sfc, double margin) {
        if (isObstacleInSFC(initial_sfc, margin)) {
            bool debug = isObstacleInSFC(initial_sfc, margin);
            throw std::invalid_argument("[CollisionConstraint] Initial SFC is occluded by obstacle");
        }

        SFC sfc, sfc_cand, sfc_update;
        std::vector<int> axis_cand = {0, 1, 2, 3, 4, 5}; // -x, -y, -z, +x, +y, +z

        int i = -1;
        int axis;
        sfc = initial_sfc;
        while (!axis_cand.empty()) {
            // initialize boxes
            sfc_cand = sfc;
            sfc_update = sfc;

            //check collision update_box only! box_current + box_update = box_cand
//                while (!isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update, margin)) {
            while (!isObstacleInSFC(sfc_update, margin) && isSFCInBoundary(sfc_update, 0)) {
                i++;
                if (i >= axis_cand.size()) {
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                sfc = sfc_cand;
                sfc_update = sfc_cand;

                //expand box_cand and get updated part of box (box_update)
                if (axis < 3) {
                    sfc_update.box_max(axis) = sfc_cand.box_min(axis);
                    sfc_cand.box_min(axis) = sfc_cand.box_min(axis) - param.world_resolution;
                    sfc_update.box_min(axis) = sfc_cand.box_min(axis);
                } else {
                    sfc_update.box_min(axis - 3) = sfc_cand.box_max(axis - 3);
                    sfc_cand.box_max(axis - 3) = sfc_cand.box_max(axis - 3) + param.world_resolution;
                    sfc_update.box_max(axis - 3) = sfc_cand.box_max(axis - 3);
                }
            }
            // if obstacle is in box then do not expand box to the current axis direction
            axis_cand.erase(axis_cand.begin() + i);
            if (i > 0) {
                i--;
            } else {
                i = axis_cand.size() - 1;
            }
        }

        return sfc;
    }
}