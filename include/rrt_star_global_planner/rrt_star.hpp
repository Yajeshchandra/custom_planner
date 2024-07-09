/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_ // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>

#include "rrt_star_global_planner/random_generator.hpp"
#include "rrt_star_global_planner/node.hpp"
#include "rrt_star_global_planner/collision_detector.hpp"

namespace rrt_star_global_planner
{

    class RRTStar
    {
    public:
        RRTStar();

        RRTStar(const std::pair<float, float> &start_point,
                const std::pair<float, float> &goal_point,
                costmap_2d::Costmap2D *costmap,
                double goal_tolerance,
                double radius,
                double epsilon,
                unsigned int max_num_nodes,
                unsigned int min_num_nodes,
                float map_width,
                float map_height);

        bool pathPlanning(std::list<std::pair<float, float>> &path); // NOLINT

        std::pair<float, float> sampleFree();

        int getNearestNodeId(const std::pair<float, float> &point);

        void createNewNode(float x, float y, int node_nearest_id);

        void chooseParent(int node_nearest_id);

        void rewire();

        std::pair<float, float> steer(float x1, float y1, float x2, float y2);

        std::vector<Node> getNodes() const;

        void computeFinalPath(std::list<std::pair<float, float>> &path); // NOLINT

        bool isGoalReached(const std::pair<float, float> &p_new);

    private:
        std::pair<float, float> start_point_;
        std::pair<float, float> goal_point_;
        costmap_2d::Costmap2D *costmap_{nullptr};
        std::vector<Node> nodes_;
        CoordinateGenerator coordinate_generator_;
        int node_count_{0};
        float map_width_;
        float map_height_;
        double radius_;
        unsigned int max_num_nodes_;
        unsigned int min_num_nodes_;
        double goal_tolerance_;
        double epsilon_;
        float origin_x_{0.0};
        float origin_y_{0.0};

        bool goal_reached_{false};

        Node goal_node_;

        CollisionDetector cd_;
    };

} // namespace rrt_star_global_planner

#endif // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_  NOLINT
