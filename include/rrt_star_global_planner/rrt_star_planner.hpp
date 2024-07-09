/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_ // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <memory>

#include "rrt_star_global_planner/node.hpp"
#include "rrt_star_global_planner/rrt_star.hpp"
#include "rrt_star_global_planner/random_generator.hpp"

namespace rrt_star_global_planner
{

    class RRTStarPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        RRTStarPlanner();

        RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        RRTStarPlanner(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);

        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan); // NOLINT

        void computeFinalPlan(std::vector<geometry_msgs::PoseStamped> &plan, // NOLINT
                              const std::list<std::pair<float, float>> &path);

    private:
        costmap_2d::Costmap2D *costmap_{nullptr};        
        bool initialized_{false};
        int max_num_nodes_;
        int min_num_nodes_;
        double epsilon_;
        float map_width_;
        float map_height_;
        double radius_;
        double goal_tolerance_;
        bool search_specific_area_{false};
        std::string global_frame_;
        std::shared_ptr<RRTStar> planner_;
    };

} // namespace rrt_star_global_planner

#endif // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
