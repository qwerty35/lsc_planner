#ifndef LSC_PLANNER_CORRIDOR_CONSTRUCTOR_HPP
#define LSC_PLANNER_CORRIDOR_CONSTRUCTOR_HPP

#include <param.hpp>
#include <mission.hpp>
#include <collision_constraints.hpp>
#include <utility>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

namespace DynamicPlanning{
    class CorridorConstructor {
    public:
        CorridorConstructor(const std::shared_ptr<DynamicEDTOctomap>& _distmap_obj,
                            const Mission& _mission,
                            const Param& _param)
                : distmap_obj(_distmap_obj), mission(_mission), param(_param) {}

        Box expandBoxFromPoint(const octomap::point3d& point, const octomap::point3d& goal_point, double agent_radius) {
            // Initialize initial_box
            std::vector<double> initial_box;
            initial_box.resize(6);

            for(int i = 0; i < 3; i++){
                double round_point_i = round(point(i % 3) / param.world_resolution) * param.world_resolution;
                if(abs(point(i) - round_point_i) < 0.01){
                    initial_box[i] = round_point_i;
                    initial_box[i + 3] = round_point_i;
                }
                else{
                    initial_box[i] = floor(point(i % 3) / param.world_resolution) * param.world_resolution;
                    initial_box[i + 3] = ceil(point(i % 3) / param.world_resolution) * param.world_resolution;
                }
            }

            if (isObstacleInBox(initial_box, agent_radius)) {
                bool debug = isObstacleInBox(initial_box, agent_radius);
                throw std::invalid_argument("[CorridorConstructor] Invalid line segment. Obstacle invades line segment.");
            }

            Box box;
            expandSFCFromBox(initial_box, goal_point, agent_radius, box);

            return box;
        }

        bool expandBoxFromPoints(const std::vector<octomap::point3d>& points,
                                 const octomap::point3d& goal_position, double agent_radius, Box& box){
            std::vector<double> initial_box = initializeBoxFromPoints(points);

            return expandSFCFromBox(initial_box, goal_position, agent_radius, box);
        }

    private:
        std::shared_ptr<DynamicEDTOctomap> distmap_obj;
        Mission mission;
        Param param;

        std::vector<double> initializeBoxFromPoints(const std::vector<octomap::point3d>& points) const{
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

        bool isObstacleInBox(const std::vector<double> &box, double margin) {
            std::array<int, 3> box_size = {0, 0, 0};
            for(int i = 0; i < 3; i++){
                box_size[i] = (int)round((box[i + 3] - box[i]) / param.world_resolution) + 1;
            }

            octomap::point3d delta;
            std::array<size_t, 3> box_iter = {0, 0, 0};
            for (box_iter[0] = 0; box_iter[0] < std::max(box_size[0], 2); box_iter[0]++) {
                for (box_iter[1] = 0; box_iter[1] < std::max(box_size[1], 2); box_iter[1]++) {
                    for (box_iter[2] = 0; box_iter[2] < std::max(box_size[2], 2); box_iter[2]++) {
                        octomap::point3d search_point;
                        for(int i = 0; i < 3; i++){
                            if(box_size[i] == 1 and box_iter[i] > 0){
                                search_point(i) = box[i];
                            }
                            else{
                                search_point(i) = box[i] + box_iter[i] * param.world_resolution;
                            }
                        }

                        // Find delta to compensate numerical error
                        for(int i = 0; i < 3; i++){
                            if (box_iter[i] == 0 && box[i] > mission.world_min(i) + SP_EPSILON_FLOAT) {
                                delta(i) = -SP_EPSILON_FLOAT;
                            }
                            else{
                                delta(i) = SP_EPSILON_FLOAT;
                            }
                        }

                        search_point = search_point + delta;
                        float dist = distmap_obj->getDistance(search_point);
                        if (dist < margin + 0.5 * param.world_resolution - SP_EPSILON_FLOAT) { // add 0.5 resolution to avoid numerical error
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        bool isBoxInBoundary(const std::vector<double> &box, double margin) {
            return box[0] > mission.world_min.x() + margin - SP_EPSILON &&
                   box[1] > mission.world_min.y() + margin - SP_EPSILON &&
                   box[2] > mission.world_min.z() + margin - SP_EPSILON &&
                   box[3] < mission.world_max.x() - margin + SP_EPSILON &&
                   box[4] < mission.world_max.y() - margin + SP_EPSILON &&
                   box[5] < mission.world_max.z() - margin + SP_EPSILON;
        }

        static bool isPointInBox(const octomap::point3d &point, const std::vector<double> &box) {
            return point.x() > box[0] - SP_EPSILON &&
                   point.y() > box[1] - SP_EPSILON &&
                   point.z() > box[2] - SP_EPSILON &&
                   point.x() < box[3] + SP_EPSILON &&
                   point.y() < box[4] + SP_EPSILON &&
                   point.z() < box[5] + SP_EPSILON;
        }

        std::vector<int> setAxisCand(const std::vector<double>& box, const octomap::point3d& goal_position) {
            std::vector<int> axis_cand;
            axis_cand.resize(6);

            octomap::point3d mid_point(0.5 * (box[0] + box[3]),
                                       0.5 * (box[1] + box[4]),
                                       0.5 * (box[2] + box[5]));
            octomap::point3d delta = goal_position - mid_point;
            std::vector<int> offsets;
            offsets.emplace_back(delta.x() > 0 ? 3 : 0);
            offsets.emplace_back(delta.y() > 0 ? 3 : 0);
            offsets.emplace_back(delta.z() > 0 ? 3 : 0);
            std::vector<double> values;
            values.emplace_back(abs(delta.x()));
            values.emplace_back(abs(delta.y()));
            values.emplace_back(abs(delta.z()));

            std::vector<int> element_order; // 0 is largest, 2 is smallest
            double max_value = -1;
            double min_value = SP_INFINITY;
            for(int i = 0; i < 3; i++){
                if(values[i] > max_value){
                    element_order.insert(element_order.begin(), i);
                    max_value = values[i];
                }
                else if(values[i] < min_value){
                    element_order.emplace_back(i);
                    min_value = values[i];
                }
                else{
                    element_order.insert(element_order.begin() + 1, i);
                }
            }

            for(int i = 0; i < 3; i++){
                axis_cand[i] = element_order[i] + offsets[element_order[i]];
                axis_cand[5 - i] = element_order[i] + (3 - offsets[element_order[i]]);
            }

            return axis_cand;
        }

        std::vector<double> expand_box(const std::vector<double>& initial_box,
                                       const octomap::point3d& goal_position, double margin) {
            std::vector<double> box, box_cand, box_update;
            std::vector<int> axis_cand = setAxisCand(initial_box, goal_position); // expandable axis direction list
//            std::vector<int> axis_cand = {0, 1, 2, 3, 4, 5};

            int i = -1;
            int axis;
            box = initial_box;
            while (!axis_cand.empty()) {
                // initialize boxes
                box_cand = box;
                box_update = box;

                //check collision update_box only! box_current + box_update = box_cand
//                while (!isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update, margin)) {
                while (!isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update, 0)) {
                    i++;
                    if (i >= axis_cand.size()) {
                        i = 0;
                    }
                    axis = axis_cand[i];

                    //update current box
                    box = box_cand;
                    box_update = box_cand;

                    //expand box_cand and get updated part of box (box_update)
                    if (axis < 3) {
                        box_update[axis + 3] = box_cand[axis];
                        box_cand[axis] = box_cand[axis] - param.world_resolution;
                        box_update[axis] = box_cand[axis];
                    } else {
                        box_update[axis - 3] = box_cand[axis];
                        box_cand[axis] = box_cand[axis] + param.world_resolution;
                        box_update[axis] = box_cand[axis];
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

            return box;
        }

        bool expandSFCFromBox(const std::vector<double>& initial_box, const octomap::point3d& goal_point,
                              double agent_radius, Box& box) {
            if (isObstacleInBox(initial_box, agent_radius)) {
                return false;
            }

            std::vector<double> expanded_box = expand_box(initial_box, goal_point, agent_radius);
            box = Box(octomap::point3d(expanded_box[0], expanded_box[1], expanded_box[2]),
                      octomap::point3d(expanded_box[3], expanded_box[4], expanded_box[5]));

            return true;
        }
    };
}

#endif //LSC_PLANNER_CORRIDOR_CONSTRUCTOR_HPP
