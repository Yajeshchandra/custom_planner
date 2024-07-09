#ifndef RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_ // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <cmath>
#include <utility>

#include "rrt_star_global_planner/node.hpp"

namespace rrt_star_global_planner
{
    
    class CollisionDetector
    {
    public:
        explicit CollisionDetector(costmap_2d::Costmap2D *costmap);

        bool isCollision(const Node &node);
        bool isCollision(const std::pair<float, float> &point);
        bool isCoordinateInBound(int mx,int my);
        bool isThereObstacleBetween(float x1, float y1, float x2, float y2);
        bool isThereObstacleBetween(const Node &node, const std::pair<float, float> &point);
        bool isThereObstacleBetween(const Node &node1, const Node &node2);
        bool worldToMap(float wx, float wy, int &mx, int &my); // NOLINT
        bool isStartAndGoalValid(const Node &start, const Node &goal);
        bool isStartAndGoalValid(const std::pair<float, float> &start, const std::pair<float, float> &goal);
        

    private:

        costmap_2d::Costmap2D *costmap_{nullptr};
        float resolution_{0.1};
        float origin_x_{0.0};
        float origin_y_{0.0};
        int size_x_{0};
        int size_y_{0};

    };

} // namespace rrt_star_global_planner

#endif // RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_  NOLINT
