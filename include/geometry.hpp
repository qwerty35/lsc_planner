#pragma once
#include <sp_const.hpp>
#include <openGJK/openGJK.hpp>

namespace DynamicPlanning{
    static ClosestPoints closestPointsBetweenPointAndLine(const point3d& point,
                                                          const point3d& line_point,
                                                          const point3d& line_direction){
        point3d a, c, line_closest_point;
        a = line_point - point;
        c = a - line_direction * a.dot(line_direction);
        line_closest_point = point + c;

        ClosestPoints closest_points;
        closest_points.dist = c.norm();
        closest_points.closest_point1 = point;
        closest_points.closest_point2 = line_closest_point;

        return closest_points;
    }

    static ClosestPoints closestPointsBetweenPointAndRay(const point3d& point,
                                                         const point3d& ray_start,
                                                         const point3d& ray_direction){
        ClosestPoints closest_points;

        point3d delta_to_start, ray_closest_point;
        delta_to_start = point - ray_start;
        if(delta_to_start.dot(ray_direction) < 0){
            closest_points.dist = delta_to_start.norm();
            closest_points.closest_point1 = point;
            closest_points.closest_point2 = ray_closest_point;
        }
        else{
            closest_points = closestPointsBetweenPointAndLine(point, ray_start, ray_direction);
        }

        return closest_points;
    }

    //closest_point: closest point in line
    static ClosestPoints closestPointsBetweenPointAndLineSegment(const point3d& point,
                                                                 const point3d& line_start,
                                                                 const point3d& line_goal){
        point3d a, b, c, n_line, rel_closest_point;
        double dist, dist_min;
        a = line_start - point;
        b = line_goal - point;

        if (a == b) {
            dist_min = a.norm();
            rel_closest_point = a;
        } else {
            dist_min = a.norm();
            rel_closest_point = a;

            dist = b.norm();
            if (dist_min > dist) {
                dist_min = dist;
                rel_closest_point = b;
            }

            n_line = (b - a).normalized();
            c = a - n_line * a.dot(n_line);
            dist = c.norm();
            if ((c - a).dot(c - b) < 0 && dist_min > dist) {
                dist_min = dist;
                rel_closest_point = c;
            }
        }

        ClosestPoints closest_points;
        closest_points.dist = dist_min;
        closest_points.closest_point1 = point;
        closest_points.closest_point2 = rel_closest_point + point;

        return closest_points;
    }

// find min((line1_start + alpha(line1_goal-line1_start) - (line2_start + alpha(line2_goal-line2_start)))
    static ClosestPoints closestPointsBetweenLinePaths(const point3d& line1_start,
                                                       const point3d& line1_goal,
                                                       const point3d& line2_start,
                                                       const point3d& line2_goal)
    {
        point3d rel_path_start, rel_path_goal, origin;
        rel_path_start = line2_start - line1_start;
        rel_path_goal = line2_goal - line1_goal;
        origin = point3d(0, 0, 0);

        ClosestPoints rel_closest_points, closest_points;
        double alpha, line2_length;
        rel_closest_points = closestPointsBetweenPointAndLineSegment(origin, rel_path_start, rel_path_goal);
        line2_length = (rel_path_goal - rel_path_start).norm();
        if(line2_length > 0){
            alpha = (rel_closest_points.closest_point2 - rel_path_start).norm() / line2_length;
        }
        else{
            alpha = 0;
        }

        closest_points.dist = rel_closest_points.dist;
        closest_points.closest_point1 = line1_start + (line1_goal - line1_start) * alpha;
        closest_points.closest_point2 = line2_start + (line2_goal - line2_start) * alpha;
        return closest_points;
    }

//TODO: change line_start, line_goal to line_point, line_direction
    static ClosestPoints closestPointsBetweenLines(const point3d& line1_start,
                                                   const point3d& line1_goal,
                                                   const point3d& line2_start,
                                                   const point3d& line2_goal)
    {
        if(line1_start == line1_goal){
            throw std::invalid_argument("[Geometry] Invalid input for distance between lines, line1 start and goal is same");
        }
        if(line2_start == line2_goal){
            throw std::invalid_argument("[Geometry] Invalid input for distance between lines, line2 start and goal is same");
        }

        ClosestPoints closest_points;

        // get closest point of line segment(a->b) from origin
        point3d n1, n2, n3, delta;
        n1 = (line1_goal - line1_start).normalized();
        n2 = (line2_goal - line2_start).normalized();
        if(n1 == n2 or n1 == -n2){
            delta = line2_start - line1_start;
            delta = delta - n1 * (delta.dot(n1));
            closest_points.dist = delta.norm();
            closest_points.closest_point1 = line1_start;
            closest_points.closest_point2 = line1_start + delta;
        }
        else{
            delta = line2_start - line1_start;
            n3 = (n2.cross(n1)).normalized();

            // get alpha1,2,3 by solving line1_start + alpha1 * n1 + alpha3 * n3 = line2_start + alpha2 * n2
            Eigen::Matrix3f A;
//            A <<  n1(0),  n1(1),  n1(2),
//                 -n2(0), -n2(1), -n2(2),
//                  n3(0),  n3(1),  n3(2);
            A <<  n1(0), -n2(0),  n3(0),
                    n1(1), -n2(1),  n3(1),
                    n1(2), -n2(2),  n3(2);
            Eigen::Vector3f b, alphas;
            b << delta(0), delta(1), delta(2);
            alphas = A.inverse() * b;

            closest_points.dist = abs(alphas(2));
            closest_points.closest_point1 = line1_start + n1 * alphas(0);
            closest_points.closest_point2 = line2_start + n2 * alphas(1);
        }
        return closest_points;
    }

    static ClosestPoints closestPointsBetweenLineSegments(const point3d& line1_start,
                                                          const point3d& line1_goal,
                                                          const point3d& line2_start,
                                                          const point3d& line2_goal)
    {
        ClosestPoints closest_point, closest_point_cand;
        closest_point.dist = SP_INFINITY;

        closest_point_cand = closestPointsBetweenPointAndLineSegment(line2_start, line1_start, line1_goal);
        if(closest_point_cand.dist < closest_point.dist){
            closest_point.dist = closest_point_cand.dist;
            closest_point.closest_point1 = closest_point_cand.closest_point2;
            closest_point.closest_point2 = line2_start;
        }

        closest_point_cand = closestPointsBetweenPointAndLineSegment(line2_goal, line1_start, line1_goal);
        if(closest_point_cand.dist < closest_point.dist){
            closest_point.dist = closest_point_cand.dist;
            closest_point.closest_point1 = closest_point_cand.closest_point2;
            closest_point.closest_point2 = line2_goal;
        }

        closest_point_cand = closestPointsBetweenPointAndLineSegment(line1_start, line2_start, line2_goal);
        if(closest_point_cand.dist < closest_point.dist){
            closest_point.dist = closest_point_cand.dist;
            closest_point.closest_point1 = line1_start;
            closest_point.closest_point2 = closest_point_cand.closest_point2;
        }

        closest_point_cand = closestPointsBetweenPointAndLineSegment(line1_goal, line2_start, line2_goal);
        if(closest_point_cand.dist < closest_point.dist){
            closest_point.dist = closest_point_cand.dist;
            closest_point.closest_point1 = line1_goal;
            closest_point.closest_point2 = closest_point_cand.closest_point2;
        }

        if(line1_start == line1_goal || line2_start == line2_goal){
            // skip
        }
        else{
            point3d n1, n2;
            n1 = (line1_goal - line1_start).normalized();
            n2 = (line2_goal - line2_start).normalized();
            if(n1 == n2 or n1 == -n2){
                // skip
            }
            else{
                closest_point_cand = closestPointsBetweenLines(line1_start, line1_goal, line2_start, line2_goal);

                //alpha check
                double alpha1, alpha2;
                alpha1 = (closest_point_cand.closest_point1 - line1_start).dot(n1) / (line1_goal - line1_start).norm();
                alpha2 = (closest_point_cand.closest_point2 - line2_start).dot(n2) / (line2_goal - line2_start).norm();
                if(closest_point_cand.dist < closest_point.dist and alpha1 < 1 and alpha1 > 0
                   and alpha2 < 1 and alpha2 > 0){
                    closest_point.dist = closest_point_cand.dist;
                    closest_point.closest_point1 = closest_point_cand.closest_point1;
                    closest_point.closest_point2 = closest_point_cand.closest_point2;
                }
            }
        }

        return closest_point;
    }

//    static lines_t getEdgesFromStaticObs(const dynamic_msgs::Obstacle& obstacle, int dimension, double z_2d = 1.0){
//        if(obstacle.type != ObstacleType::STATICOBSTACLE or obstacle.dimensions.size() != 3 ){
//            throw std::invalid_argument("[Geometry] failed to get closest point to static obstacle, Invalid obstacle");
//        }
//
//        lines_t edges;
//        std::array<double, 2> x_bound, y_bound, z_bound;
//        x_bound = {obstacle.pose.position.x - obstacle.dimensions[0],
//                   obstacle.pose.position.x + obstacle.dimensions[0]};
//        y_bound = {obstacle.pose.position.y - obstacle.dimensions[1],
//                   obstacle.pose.position.y + obstacle.dimensions[1]};
//        if(dimension == 2){
//            z_bound = {z_2d, z_2d};
//        }
//        else{
//            z_bound = {obstacle.pose.position.z - obstacle.dimensions[2],
//                       obstacle.pose.position.z + obstacle.dimensions[2]};
//        }
//
//        // find edges
//        point3d vertex1, vertex2;
//        for(int i = 0; i < 2; i++){
//            for(int j = 0; j < 2; j++){
//                for(int k = 0; k < dimension - 1; k++){
//                    vertex1 = point3d(x_bound[i], y_bound[j], z_bound[k]);
//
//                    if(i == 0){
//                        vertex2 = point3d(x_bound[i + 1], y_bound[j], z_bound[k]);
//                        edges.emplace_back(Line(vertex1, vertex2));
//                    }
//                    if(j == 0){
//                        vertex2 = point3d(x_bound[i], y_bound[j+1], z_bound[k]);
//                        edges.emplace_back(Line(vertex1, vertex2));
//                    }
//                    if(dimension == 3 and k == 0){
//                        vertex2 = point3d(x_bound[i+1], y_bound[j], z_bound[k+1]);
//                        edges.emplace_back(Line(vertex1, vertex2));
//                    }
//                }
//            }
//        }
//
//        return edges;
//    }

//    static lines_t getInflatedEdgesFromStaticObs(const dynamic_msgs::Obstacle& obstacle, double radius,
//                                                 int dimension, double z_2d = 1.0){
//        if(obstacle.type != ObstacleType::STATICOBSTACLE or obstacle.dimensions.size() != 3 or dimension == 3){
//            throw std::invalid_argument("[Geometry] failed to get closest point to static obstacle, Invalid obstacle");
//        }
//
//        lines_t edges;
//        std::array<double, 2> x_bound, y_bound;
//        x_bound = {obstacle.pose.position.x - obstacle.dimensions[0],
//                   obstacle.pose.position.x + obstacle.dimensions[0]};
//        y_bound = {obstacle.pose.position.y - obstacle.dimensions[1],
//                   obstacle.pose.position.y + obstacle.dimensions[1]};
//
//        // find edges
//        point3d vertex1, vertex2;
//        for(int i = 0; i < 2; i++){
//            for(int j = 0; j < 2; j++){
//                vertex1 = point3d(x_bound[i], y_bound[j], z_2d);
//
//                if(i == 0){
//                    vertex2 = point3d(x_bound[i+1], y_bound[j], z_2d);
//                    if(j == 0){
//                        edges.emplace_back(Line(vertex2 + point3d(0, -radius, 0),
//                                                vertex1 + point3d(0, -radius, 0)));
//                    }
//                    else{
//                        edges.emplace_back(Line(vertex1 + point3d(0, radius, 0),
//                                                vertex2 + point3d(0, radius, 0)));
//                    }
//                }
//                if(j == 0){
//                    vertex2 = point3d(x_bound[i], y_bound[j+1], z_2d);
//                    if(i == 0){
//                        edges.emplace_back(Line(vertex1 + point3d(-radius, 0, 0),
//                                                vertex2 + point3d(-radius, 0, 0)));
//                    }
//                    else{
//                        edges.emplace_back(Line(vertex2 + point3d(radius, 0, 0),
//                                                vertex1 + point3d(radius, 0, 0)));
//                    }
//                }
//            }
//        }
//
//        return edges;
//    }

//    static ClosestPoints closestPointsBetweenPointAndStaticObs(const point3d& point,
//                                                               const dynamic_msgs::Obstacle& obstacle,
//                                                               int dimension, double z_2d = 1.0)
//    {
//        if(obstacle.type != ObstacleType::STATICOBSTACLE or obstacle.dimensions.size() != 3 ){
//            throw std::invalid_argument("[Geometry] failed to get closest point to static obstacle, Invalid obstacle");
//        }
//
//        float x, y, z, x_min, x_max, y_min, y_max, z_min, z_max;
//        x_min = obstacle.pose.position.x - obstacle.dimensions[0];
//        x_max = obstacle.pose.position.x + obstacle.dimensions[0];
//        y_min = obstacle.pose.position.y - obstacle.dimensions[1];
//        y_max = obstacle.pose.position.y + obstacle.dimensions[1];
//        z_min = obstacle.pose.position.z - obstacle.dimensions[2];
//        z_max = obstacle.pose.position.z + obstacle.dimensions[2];
//
//        x = std::min(x_max, std::max(point.x(), x_min));
//        y = std::min(y_max, std::max(point.y(), y_min));
//        z = std::min(z_max, std::max(point.z(), z_min));
//
//        ClosestPoints closest_points;
//        if(dimension == 2){
//            closest_points.closest_point1 = point;
//            closest_points.closest_point2 = point3d(x, y, z_2d);
//            closest_points.dist = (closest_points.closest_point2 - closest_points.closest_point1).norm();
//        }
//        else{
//            closest_points.closest_point1 = point;
//            closest_points.closest_point2 = point3d(x, y, z);
//            closest_points.dist = (closest_points.closest_point2 - closest_points.closest_point1).norm();
//        }
//
//        return closest_points;
//    }

    static ClosestPoints closestPointsBetweenPointAndConvexHull(const point3d& point,
                                                                const points_t& convex_hull){
        /* Squared distance computed by openGJK.                                 */
        double dd;
        /* Structure of simplex used by openGJK.                                 */
        struct simplex  s;
        double v[3];
        /* Number of vertices defining body 1 and body 2, respectively.          */
        int nvrtx1, nvrtx2;
        /* Structures of body 1 and body 2, respectively.                        */
        struct bd bd1;
        struct bd bd2;

        bd1.numpoints = static_cast<int>(convex_hull.size());
        bd1.coord = point3DsToArray(convex_hull);

        points_t point_vector;
        point_vector.emplace_back(point);
        bd2.numpoints = 1;
        bd2.coord = point3DsToArray(point_vector);

        // Run GJK algorithm
        dd = gjk (bd1, bd2, &s, v);

        ClosestPoints closest_points;
        closest_points.closest_point1 = point;
        closest_points.closest_point2 = point + point3d(v[0], v[1], v[2]);
        closest_points.dist = dd;

        return closest_points;
    }

//    static ClosestPoints closestPointsBetweenLineSegmentAndStaticObs(const point3d& start_point,
//                                                                     const point3d& goal_point,
//                                                                     const dynamic_msgs::Obstacle& obstacle,
//                                                                     int dimension, double z_2d = 1.0)
//    {
//        if(obstacle.type != ObstacleType::STATICOBSTACLE or obstacle.dimensions.size() != 3 ){
//            throw std::invalid_argument("[Geometry] failed to get closest point to static obstacle, Invalid obstacle");
//        }
//
//        ClosestPoints closest_points;
//        if(isPointInStaticObs(start_point, obstacle, dimension)){
//            closest_points.dist = 0;
//            closest_points.closest_point1 = start_point;
//            closest_points.closest_point2 = start_point;
//            return closest_points;
//        }
//
//        //find closest point to 12 edges
//        lines_t edges = getEdgesFromStaticObs(obstacle, dimension, z_2d);
//
//        ClosestPoints closest_points_cand;
//        closest_points.dist = SP_INFINITY;
//        if(dimension == 2){
//            for(auto& edge : edges){
//                //TODO: plane check?
//                //TODO: available only when dimension == 2
//                closest_points_cand = closestPointsBetweenLineSegments(start_point, goal_point,
//                                                                       edge.start_point, edge.end_point);
//                if(closest_points_cand.dist < closest_points.dist){
//                    closest_points.dist = closest_points_cand.dist;
//                    closest_points.closest_point1 = closest_points_cand.closest_point1;
//                    closest_points.closest_point2 = closest_points_cand.closest_point2;
//                }
//            }
//        }
//        else{
//            //TODO: implement this part!
//            throw std::invalid_argument("[Geometry] not implemented yet");
//        }
//
//        return closest_points;
//    }

//    static point3d closestPointsBetweenPointAndStaticObs(const dynamic_msgs::Obstacle& obstacle,
//                                                                  const point3d& point,
//                                                                  int dimension, double world_z_2d = 1.0)
//    {
//        point3d closest_point_in_obs, closest_point_in_line;
//        double dist = distanceBetweenPointAndStaticObs(obstacle, point, closest_point_in_line, dimension, world_z_2d);
//        return closest_point_in_obs;
//    }

//    static point3d getClosestPointOnStaticObstacle(const dynamic_msgs::Obstacle& obstacle,
//                                                            const point3d& start_point,
//                                                            const point3d& goal_point,
//                                                            int dimension, double world_z_2d = 1.0)
//    {
//        point3d closest_point_in_obs, closest_point_in_line;
//        double dist = distanceBetweenLineSegmentAndStaticObs(obstacle, start_point, goal_point,
//                                                             closest_point_in_obs, closest_point_in_line,
//                                                             dimension, world_z_2d);
//
//        return closest_point_in_obs;
//    }

//    static point3d getClosestVertexOnStaticObstacle(const dynamic_msgs::Obstacle& obstacle,
//                                                             const point3d& start_point,
//                                                             const point3d& goal_point,
//                                                             int dimension, double world_z_2d = 1.0)
//    {
//        std::array<double, 2> x_bound, y_bound, z_bound;
//        x_bound = {obstacle.pose.position.x - obstacle.dimensions[0],
//                   obstacle.pose.position.x + obstacle.dimensions[0]};
//        y_bound = {obstacle.pose.position.y - obstacle.dimensions[1],
//                   obstacle.pose.position.y + obstacle.dimensions[1]};
//        if(dimension == 2){
//            z_bound = {world_z_2d, world_z_2d};
//        }
//        else{
//            z_bound = {obstacle.pose.position.z - obstacle.dimensions[2],
//                       obstacle.pose.position.z + obstacle.dimensions[2]};
//        }
//
//        point3d vertex, closest_vertex;
//        double dist, min_dist = SP_INFINITY;
//        for(int i = 0; i < 2; i++){
//            for(int j = 0; j < 2; j++){
//                for(int k = 0; k < dimension - 1; k++){
//                    vertex = point3d(x_bound[i], y_bound[j], z_bound[k]);
//                    dist = distanceBetweenPointAndLineSegment(start_point, goal_point, vertex);
//                    if(dist < min_dist){
//                        min_dist = dist;
//                        closest_vertex = vertex;
//                    }
//                }
//            }
//        }
//
//        return closest_vertex;
//    }

//    static bool checkCollisionBetweenLineSegmentAndBox(const Obstacle& obstacle,
//                                                       point3d start_point, point3d goal_point,
//                                                       double radius, int dimension, double z_2d = 1.0){
//        if(obstacle.type != ObstacleType::STATICOBSTACLE or obstacle.dimensions.size() != 3 ){
//            throw std::invalid_argument("[Geometry] collision check between line and box failed, Invalid obstacle");
//        }
//
//        std::array<double,3> box_min, box_max;
////        box_min = {obstacle.pose.position.x - obstacle.dimensions[0],
////                   obstacle.pose.position.y - obstacle.dimensions[1],
////                   obstacle.pose.position.z - obstacle.dimensions[2]};
////        box_max = {obstacle.pose.position.x + obstacle.dimensions[0],
////                   obstacle.pose.position.y + obstacle.dimensions[1],
////                   obstacle.pose.position.z + obstacle.dimensions[2]};
//        box_min = {obstacle.position.x - obstacle.dimensions[0] - radius,
//                   obstacle.position.y - obstacle.dimensions[1] - radius,
//                   obstacle.position.z - obstacle.dimensions[2] - radius};
//        box_max = {obstacle.position.x + obstacle.dimensions[0] + radius,
//                   obstacle.position.y + obstacle.dimensions[1] + radius,
//                   obstacle.position.z + obstacle.dimensions[2] + radius};
//
//        std::array<double,3>  start, goal;
//        start = {start_point.x(), start_point.y(), start_point.z()};
//        goal = {goal_point.x(), goal_point.y(), goal_point.z()};
//
//        double alpha_min, alpha_max, alpha_cand_min, alpha_cand_max;
//        alpha_min = 0.0;
//        alpha_max = 1.0;
//
//        for(int i = 0; i < dimension; i++) {
//            if (start[i] != goal[i]) {
//                alpha_cand_min = (box_min[i] - start[i]) / (goal[i] - start[i]);
//                alpha_cand_max = (box_max[i] - start[i]) / (goal[i] - start[i]);
//                if (alpha_cand_max < alpha_cand_min) {
//                    std::swap(alpha_cand_min, alpha_cand_max);
//                }
//
//                if (alpha_min > alpha_cand_max or alpha_max < alpha_cand_min) {
//                    return false;
//                } else {
//                    alpha_min = std::max(alpha_min, alpha_cand_min);
//                    alpha_max = std::min(alpha_max, alpha_cand_max);
//                }
//            } else if (start[i] < box_min[i] or start[i] > box_max[i]) {
//                return false;
//            }
//        }
//
//        // edge check
//        ClosestPoints closest_points;
//        closest_points = closestPointsBetweenLineSegmentAndStaticObs(start_point, goal_point,
//                                                                     obstacle, dimension, z_2d);
//
//        return closest_points.dist < radius;
//    }

    static double computeCollisionTime(const point3d &obs_start, const point3d &obs_goal,
                                       const point3d &agent_start, const point3d &agent_goal,
                                       double collision_radius, double time_horizon) {
        //TODO: just solve 2nd poly?
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenLinePaths(obs_start, obs_goal, agent_start, agent_goal);

        double dist_in_sphere1, dist_in_sphere2, collision_time = SP_INFINITY;
        if (closest_points.dist <= collision_radius) {
            point3d a, b, delta;
            a = agent_start - obs_start;
            b = agent_goal - obs_goal;
            delta = closest_points.closest_point2 - closest_points.closest_point1;

            if (a.norm() <= collision_radius) {
                collision_time = 0;
            } else if (delta == b) {
                point3d n_line, c;
                double dist_to_b, dist_to_c;
                dist_to_b = b.norm();
                n_line = (b - a).normalized();
                c = a - n_line * a.dot(n_line);
                dist_to_c = c.norm();
                dist_in_sphere1 = sqrt(collision_radius * collision_radius - dist_to_c * dist_to_c);
                dist_in_sphere2 = sqrt(dist_to_b * dist_to_b - dist_to_c * dist_to_c);
                collision_time = (1 - (dist_in_sphere1 - dist_in_sphere2) / (b - a).norm()) * time_horizon;
            } else {
                double dist_to_b = b.norm();
                dist_in_sphere1 = sqrt(collision_radius * collision_radius - closest_points.dist * closest_points.dist);
                dist_in_sphere2 = sqrt(dist_to_b * dist_to_b - closest_points.dist * closest_points.dist);
                collision_time = (1 - (dist_in_sphere1 + dist_in_sphere2) / (b - a).norm()) * time_horizon;
            }
        }

        return collision_time;
    }

//    static double computeCollisionTime(const dynamic_msgs::Obstacle &obstacle,
//                                       const point3d &start_point, const point3d &goal_point,
//                                       double collision_radius, double time_horizon, int dimension, double z_2d) {
//        if (obstacle.type != ObstacleType::STATICOBSTACLE or obstacle.dimensions.size() != 3) {
//            throw std::invalid_argument("[Geometry] collision check between line and box failed, Invalid obstacle");
//        }
//
//        std::array<double, 3> big_box_min, big_box_max;
//        big_box_min = {obstacle.pose.position.x - obstacle.dimensions[0] - collision_radius,
//                       obstacle.pose.position.y - obstacle.dimensions[1] - collision_radius,
//                       obstacle.pose.position.z - obstacle.dimensions[2] - collision_radius};
//        big_box_max = {obstacle.pose.position.x + obstacle.dimensions[0] + collision_radius,
//                       obstacle.pose.position.y + obstacle.dimensions[1] + collision_radius,
//                       obstacle.pose.position.z + obstacle.dimensions[2] + collision_radius};
//
//        std::array<double, 3> start, goal;
//        start = {start_point.x(), start_point.y(), start_point.z()};
//        goal = {goal_point.x(), goal_point.y(), goal_point.z()};
//
//        double alpha_min, alpha_max, alpha_cand_min, alpha_cand_max;
//        alpha_min = 0.0;
//        alpha_max = 1.0;
//
//        for (int i = 0; i < dimension; i++) {
//            if (start[i] != goal[i]) {
//                alpha_cand_min = (big_box_min[i] - start[i]) / (goal[i] - start[i]);
//                alpha_cand_max = (big_box_max[i] - start[i]) / (goal[i] - start[i]);
//                if (alpha_cand_max < alpha_cand_min) {
//                    std::swap(alpha_cand_min, alpha_cand_max);
//                }
//
//                if (alpha_min > alpha_cand_max or alpha_max < alpha_cand_min) {
//                    return SP_INFINITY;
//                } else {
//                    alpha_min = std::max(alpha_min, alpha_cand_min);
//                    alpha_max = std::min(alpha_max, alpha_cand_max);
//                }
//            } else if (start[i] < big_box_min[i] or start[i] > big_box_max[i]) {
//                return SP_INFINITY;
//            }
//        }
//
//        point3d closest_point_cand = start_point + (goal_point - start_point) * alpha_min;
//        ClosestPoints closest_points_to_static_obs = closestPointsBetweenPointAndStaticObs(closest_point_cand,
//                                                                                           obstacle,
//                                                                                           dimension, z_2d);
//
//        point3d closest_point_obs = closest_points_to_static_obs.closest_point2;
//        double collision_time = computeCollisionTime(closest_point_obs, closest_point_obs,
//                                                     start_point, goal_point,
//                                                     collision_radius, time_horizon);
//        return collision_time;
//    }

    static point3d lineDirection(const Line& line){
        if(line.start_point == line.end_point){
            throw std::invalid_argument("[Geometry] line start and end is the same");
        }
        return (line.end_point - line.start_point).normalized();
    }

    static double safeDistInDirection(const point3d& position, const point3d& direction,
                                      const std::vector<dynamic_msgs::Obstacle>& obstacles, double radius,
                                      int dimension, double z_2d) {
        double radius_sum, dist_to_closest_point, safe_dist = SP_INFINITY;
        point3d obs_position, ray_closest_point;
        ClosestPoints closest_points;
//        for (int oi = 0; oi < obstacles.size(); oi++) {
//            if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
//                double collision_time = computeCollisionTime(obstacles[oi], position, position + direction * 10.0, radius, 1.0, dimension);
//                closest_points = closestPointsBetweenPointAndStaticObs(position, obstacles[oi], dimension, world_z_2d); //TODO: not correct code!
//                obs_position = closest_points.closest_point2;
//                radius_sum = radius;
//            } else {
//                obs_position = pointMsgToPoint3d(obstacles[oi].pose.position);
//                radius_sum = obstacles[oi].radius + radius;
//            }
//
//            closest_points = closestPointsBetweenPointAndRay(obs_position, position, direction);
//            ray_closest_point = closest_points.closest_point2;
//            if (closest_points.dist < radius_sum) {
//                dist_to_closest_point = (ray_closest_point - position).norm();
//                safe_dist_cand = std::max(dist_to_closest_point -
//                                          sqrt(radius_sum * radius_sum - closest_points.dist * closest_points.dist),
//                                          0.0);
//                if (safe_dist_cand < safe_dist) {
//                    safe_dist = safe_dist_cand;
//                }
//            }
//        }

        for (int oi = 0; oi < obstacles.size(); oi++) {
            double safe_dist_cand = SP_INFINITY;
//            if (obstacles[oi].type == ObstacleType::STATICOBSTACLE) {
//                double fake_dist_to_goal = 10.0;
//                double fake_horizon = 1.0;
//                double collision_time = computeCollisionTime(obstacles[oi], position,
//                                                             position + direction * fake_dist_to_goal, radius,
//                                                             fake_horizon, dimension, z_2d);
//                safe_dist_cand = fake_dist_to_goal * collision_time / fake_horizon;
//            } else
            {
                obs_position = pointMsgToPoint3d(obstacles[oi].pose.position);
                radius_sum = obstacles[oi].radius + radius;
                closest_points = closestPointsBetweenPointAndRay(obs_position, position, direction);
                ray_closest_point = closest_points.closest_point2;
                if (closest_points.dist < radius_sum) {
                    dist_to_closest_point = (ray_closest_point - position).norm();
                    safe_dist_cand = std::max(dist_to_closest_point -
                                              sqrt(radius_sum * radius_sum - closest_points.dist * closest_points.dist),
                                              0.0);
                }
            }
            if (safe_dist_cand < safe_dist) {
                safe_dist = safe_dist_cand;
            }
        }

        return safe_dist;
    }
}