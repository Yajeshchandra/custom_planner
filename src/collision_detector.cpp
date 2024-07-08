/*
  Copyright 2021 - Rafael Barreto
*/
#include <ros/ros.h>
#include "rrt_star_global_planner/collision_detector.hpp"

namespace rrt_star_global_planner {

CollisionDetector::CollisionDetector(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
  if (costmap_ != nullptr) {
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
  }
}

bool CollisionDetector::isThisPointCollides(float wx, float wy) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // no collision
    return false;
  }

  int mx, my;
  worldToMap(wx, wy, mx, my);
  ROS_INFO("M-Point: (%d, %d)", mx, my);
  ROS_INFO("Cost Map COst: %d", costmap_->getCost(mx, my));

  if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
    ROS_INFO("x:%d",costmap_->getSizeInCellsX());
    ROS_INFO("y:%d",costmap_->getSizeInCellsY());
    ROS_INFO("Point out of costmap bounds!");
    return true;
  ROS_INFO("Pppppppppppppppppppppppppp!");
  // getCost returns unsigned char
  int cost = int(costmap_->getCost(mx, my));
  ROS_INFO("Cost: %d", cost);

  if (cost > 250)
    ROS_INFO("true");
    return true;

  return false;
}

bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) {
    // there is NO obstacles
    return false;
  }

  float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
  if (dist < resolution_) {
    return (isThisPointCollides(point.first, point.second)) ? true : false;
  } else {
    int steps_number = static_cast<int>(floor(dist/resolution_));
    float theta = atan2(node.y - point.second, node.x - point.first);
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = node.x + n*resolution_*cos(theta);
      p_n.second = node.y + n*resolution_*sin(theta);
      if (isThisPointCollides(p_n.first, p_n.second))
        return true;
    }
    return false;
  }
}

bool CollisionDetector::isThereObstacleBetween(const Node &node1, const Node &node2) {
  return isThereObstacleBetween(node1, std::make_pair(node2.x, node2.y));
}

void CollisionDetector::worldToMap(float wx, float wy, int& mx, int& my) {
  if (costmap_ != nullptr) {
    mx = (wx - origin_x_) / resolution_;
    my = (wy - origin_y_) / resolution_;
  }
}

}  // namespace rrt_star_global_planner
