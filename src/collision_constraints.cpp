#include <collision_constraints.hpp>

namespace DynamicPlanning{
    LSC::LSC(const octomap::point3d& _obs_control_point,
             const octomap::point3d& _normal_vector,
             double _d)
            : obs_control_point(_obs_control_point), normal_vector(_normal_vector), d(_d) {}

    visualization_msgs::Marker LSC::convertToMarker(double agent_radius) const{
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = "world"; //TODO: frame id parameterization
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


    Box::Box(const octomap::point3d& _box_min, const octomap::point3d& _box_max){
        box_min = _box_min;
        box_max = _box_max;
    }

    LSCs Box::convertToLSCs(int dim) const{
        octomap::point3d normal_vector_min, normal_vector_max;
        octomap::point3d zero_point = octomap::point3d(0, 0, 0);
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

    visualization_msgs::Marker Box::convertToMarker(double agent_radius) const{
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = "world"; //TODO: frame id
        msg_marker.type = visualization_msgs::Marker::LINE_LIST;
        msg_marker.action = visualization_msgs::Marker::ADD;
        msg_marker.pose.position = defaultPoint();
        msg_marker.pose.orientation = defaultQuaternion();
        msg_marker.scale.x = 0.03;

        octomap::point3d inflation_vector(agent_radius, agent_radius, agent_radius);
        Box inflated_box = Box(box_min - inflation_vector, box_max + inflation_vector);
        lines_t edges = inflated_box.getEdges();
        for(const auto& edge : edges){
            msg_marker.points.emplace_back(point3DToPointMsg(edge.start_point));
            msg_marker.points.emplace_back(point3DToPointMsg(edge.end_point));
        }

        return msg_marker;
    }

    bool Box::isPointInBox(const octomap::point3d& point) const{
        return point.x() > box_min.x() - SP_EPSILON_FLOAT &&
               point.y() > box_min.y() - SP_EPSILON_FLOAT &&
               point.z() > box_min.z() - SP_EPSILON_FLOAT &&
               point.x() < box_max.x() + SP_EPSILON_FLOAT &&
               point.y() < box_max.y() + SP_EPSILON_FLOAT &&
               point.z() < box_max.z() + SP_EPSILON_FLOAT;
    }

    bool Box::isPointsInTwoBox(const std::vector<octomap::point3d>& points, const Box& other_sfc) const{
        if(not intersectWith(other_sfc)){
            return false;
        }

        for(const auto& point : points){
            if(!isPointInBox(point) && !other_sfc.isPointInBox(point)){
                return false;
            }
        }
        return true;
    }

    bool Box::isSuperSetOfConvexHull(const std::vector<octomap::point3d>& convex_hull) const{
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

    bool Box::isLineOnBoundary(const Line& line) const {
        std::vector<octomap::point3d> points = {line.start_point, line.end_point};
        if (not isSuperSetOfConvexHull(points)) {
            return false;
        }

        for (int i = 0; i < 3; i++) {
            if (abs(line.start_point(i) - line.end_point(i)) < SP_EPSILON_FLOAT and
                (abs(line.start_point(i) - box_min(i)) < SP_EPSILON_FLOAT or
                 abs(line.start_point(i) - box_max(i)) < SP_EPSILON_FLOAT)) {
                return true;
            }
        }

        return false;
    }

    bool Box::intersectWith(const Box& other_sfc) const{
        Box inter_sfc = intersection(other_sfc);
        for(int i = 0; i <3; i++){
            if(inter_sfc.box_min(i) > inter_sfc.box_max(i) - SP_EPSILON_FLOAT){
                return false;
            }
        }
        return true;
    }

    Box Box::unify(const Box& other_sfc) const{
        Box unified_box;
        for(int i = 0; i < 3; i++){
            unified_box.box_min(i) = std::min(box_min(i), other_sfc.box_min(i));
            unified_box.box_max(i) = std::max(box_max(i), other_sfc.box_max(i));
        }
        return unified_box;
    }

    Box Box::intersection(const Box& other_sfc) const{
        Box inter_box;
        for(int i = 0; i < 3; i++){
            inter_box.box_min(i) = std::max(box_min(i), other_sfc.box_min(i));
            inter_box.box_max(i) = std::min(box_max(i), other_sfc.box_max(i));
        }
        return inter_box;
    }

    std::vector<octomap::point3d> Box::getVertices() const{
        std::vector<octomap::point3d> vertices;
        vertices.emplace_back(box_min);
        vertices.emplace_back(box_max);
        for(int i = 0; i < 3; i++){
            octomap::point3d point1 = box_min;
            octomap::point3d point2 = box_max;
            point1(i) = box_max(i);
            point2(i) = box_min(i);
            vertices.emplace_back(point1);
            vertices.emplace_back(point2);
        }

        return vertices;
    }

    lines_t Box::getEdges() const{
        lines_t edges;

        // find edges
        octomap::point3d vertex1, vertex2, vertex3;

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

    octomap::point3d Box::getBoxMin() const{
        return box_min;
    }

    octomap::point3d Box::getBoxMax() const{
        return box_max;
    }

    LSCs SFC::convertToLSCs(int dim) const{
        LSCs ret = box.convertToLSCs(dim);
        ret.insert(ret.end(), lscs.begin(), lscs.end());
        return ret;
    }

    bool SFC::update(const Box& box_){
        box = box_;
        inter_box = box_;
        lscs.clear();
        return true;
    }

    bool SFC::update(const std::vector<octomap::point3d>& convex_hull, const Box& box1, const Box& box2) {
        double dist_to_convex_hull = 0;
        return update(convex_hull, box1, box2, dist_to_convex_hull);
    }


    bool SFC::update(const std::vector<octomap::point3d>& convex_hull,
                     const Box& box1, const Box& box2, double& dist_to_convex_hull) {
        box = box1.unify(box2);
        inter_box = box1.intersection(box2);
        lscs.clear();

        if(not box1.intersectWith(box2)){
            return false;
        }

        if(not box1.isPointsInTwoBox(convex_hull, box2)){
            return false;
        }

        lines_t edges = inter_box.getEdges();
        for(const auto& edge : edges){
            if(!box1.isLineOnBoundary(edge) or
               !box2.isLineOnBoundary(edge) or
               box.isLineOnBoundary(edge))
            {
                continue;
            }

            std::vector<octomap::point3d> proj_convex_hull;
            octomap::point3d edge_direction = edge.direction();
            for(const auto& point : convex_hull){
                octomap::point3d rel_point = point - edge.start_point;
                octomap::point3d proj_point = rel_point - edge_direction * (edge_direction.dot(rel_point));
                proj_convex_hull.emplace_back(proj_point);
            }
            std::vector<octomap::point3d> vertices = inter_box.getVertices();
            for(const auto& vertex : vertices){
                if((edge.start_point - vertex).norm_sq() < SP_EPSILON_FLOAT or
                   (edge.end_point - vertex).norm_sq() < SP_EPSILON_FLOAT){
                    continue;
                }
                octomap::point3d rel_point = vertex - edge.start_point;
                octomap::point3d proj_point = rel_point - edge_direction * (edge_direction.dot(rel_point));
                proj_convex_hull.emplace_back(proj_point);
            }

            ClosestPoints closest_points = closestPointsBetweenPointAndConvexHull(octomap::point3d(0,0,0),
                                                                                  proj_convex_hull);
            dist_to_convex_hull = -closest_points.dist;

            if(closest_points.dist > SP_EPSILON_FLOAT){
                octomap::point3d normal_vector = (closest_points.closest_point2 - closest_points.closest_point1).normalized();

                LSC lsc(edge.start_point, normal_vector, 0);
                lscs.emplace_back(lsc);
            }
            else{
                octomap::point3d normal_vector, normal_vector_cand;
                for(const auto& proj_point : proj_convex_hull){
                    if(proj_point.norm_sq() < SP_EPSILON_FLOAT){
                        continue;
                    }

                    normal_vector_cand = proj_point.cross(edge_direction).normalized();
                    bool isProperNormalVector = true;
                    for(const auto& point_j : proj_convex_hull){
                        if(point_j.dot(normal_vector_cand) < -SP_EPSILON_FLOAT){
                            isProperNormalVector = false;
                            break;
                        }
                    }
                    if(not isProperNormalVector){
                        isProperNormalVector = true;
                        normal_vector_cand = -normal_vector_cand;
                        for(const auto& point_j : proj_convex_hull){
                            if(point_j.dot(normal_vector_cand) < -SP_EPSILON_FLOAT){
                                isProperNormalVector = false;
                                break;
                            }
                        }
                    }

                    if(isProperNormalVector){
                        normal_vector = (normal_vector + normal_vector_cand).normalized();
                        break;
                    }
                }
                if(normal_vector.norm() == 0){
                    // convex hull is out of two boxes
                    return false;
                }

                LSC sfce(edge.start_point, normal_vector, 0);
                lscs.emplace_back(sfce);
            }
        }

        return true;
    }

    void CollisionConstraints::initialize(int N_obs_, int M_, int n_, double dt_, std::set<int> obs_slack_indices_){
        N_obs = N_obs_;
        M = M_;
        n = n_;
        dt = dt_;
        obs_slack_indices = std::move(obs_slack_indices_);

        //RSFC
        lscs.resize(N_obs);
        for(int oi = 0; oi < N_obs; oi++){
            lscs[oi].resize(M);
            for(int m = 0; m < M; m++){
                lscs[oi][m].resize(n + 1);
            }
        }

        //SFC
        sfcs.resize(M);
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

    void CollisionConstraints::setLSC(int oi, int m,
                                      const std::vector<octomap::point3d>& obs_control_points,
                                      const octomap::point3d& normal_vector,
                                      const std::vector<double>& ds) {
        for(int i = 0; i < n + 1; i++){
            lscs[oi][m][i] = LSC(obs_control_points[i], normal_vector, ds[i]);
        }
    }

    void CollisionConstraints::setLSC(int oi, int m,
                                      const std::vector<octomap::point3d>& obs_control_points,
                                      const octomap::point3d& normal_vector,
                                      double d) {
        for(int i = 0; i < n + 1; i++){
            lscs[oi][m][i] = LSC(obs_control_points[i], normal_vector, d);
        }
    }

    void CollisionConstraints::setSFC(int m, const SFC& sfc){
        sfcs[m] = sfc;
    }

    void CollisionConstraints::setSFC(int m, const Box& box){
        sfcs[m].box = box;
        sfcs[m].lscs.clear();
        sfcs[m].box_library_idx1 = -1;
        sfcs[m].box_library_idx2 = -1;
    }

    visualization_msgs::MarkerArray CollisionConstraints::convertToMarkerArrayMsg (
            const std::vector<dynamic_msgs::Obstacle>& obstacles,
            const std::vector<std_msgs::ColorRGBA>& colors,
            int agent_id, double agent_radius) const
    {
        visualization_msgs::MarkerArray msg1 = convertLSCsToMarkerArrayMsg(obstacles, colors, agent_radius);
        visualization_msgs::MarkerArray msg2 = convertSFCsToMarkerArrayMsg(colors[agent_id], agent_radius);
        msg1.markers.insert(msg1.markers.end(), msg2.markers.begin(), msg2.markers.end());
        return msg1;
    }

    visualization_msgs::MarkerArray CollisionConstraints::convertLSCsToMarkerArrayMsg (
            const std::vector<dynamic_msgs::Obstacle>& obstacles,
            const std::vector<std_msgs::ColorRGBA>& colors,
            double agent_radius) const
    {
        visualization_msgs::MarkerArray msg_marker_array;

        if (lscs.empty()) {
            return msg_marker_array;
        }

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
                visualization_msgs::Marker msg_marker = lscs[oi][m][0].convertToMarker(0);
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
            visualization_msgs::Marker msg_marker = sfcs[m].box.convertToMarker(0);
            msg_marker.id = m;
            msg_marker.ns = "SFC" + std::to_string(m);
            msg_marker.color = color;
            msg_marker.color.a = 1.0;
            msg_marker_array.markers.emplace_back(msg_marker);
        }

        for(int m = 0; m < M; m++){
            int marker_id = 2*M;
            for(const auto& lsc : sfcs[m].lscs){
//                visualization_msgs::Marker msg_marker = sfce.convertToMarker(agent_radius);
                visualization_msgs::Marker msg_marker = lsc.convertToMarker(0);
                msg_marker.id = marker_id;
                msg_marker.ns = "LSC" + std::to_string(m);
                msg_marker.color = color;
                msg_marker.color.a = 0.2;
                msg_marker_array.markers.emplace_back(msg_marker);
                marker_id++;
            }

            // delete sfcs used in the previous step
            int max_n_lsc = 12;
            for(int i = marker_id; i < M + max_n_lsc; i++){
                visualization_msgs::Marker msg_marker;
                msg_marker.action = visualization_msgs::Marker::DELETE;
                msg_marker.id = i;
                msg_marker.ns = "LSC" + std::to_string(m);
                msg_marker_array.markers.emplace_back(msg_marker);
            }
        }

        return msg_marker_array;
    }

    dynamic_msgs::CollisionConstraint CollisionConstraints::convertToRawMsg(
            const std::vector<dynamic_msgs::Obstacle>& obstacles, int planner_seq) const {
        dynamic_msgs::CollisionConstraint msg_collision_constraint;
        msg_collision_constraint.planner_seq = planner_seq;

        dynamic_msgs::Param constraint_param;
        constraint_param.M = M;
        constraint_param.n = n;
        constraint_param.dt = dt;

        if(not lscs.empty()){
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
                msg_collision_constraint.sfcs[m].box_min = point3DToPointMsg(sfcs[m].box.getBoxMin());
                msg_collision_constraint.sfcs[m].box_max = point3DToPointMsg(sfcs[m].box.getBoxMax());
            }
        }

        return msg_collision_constraint;
    }
}