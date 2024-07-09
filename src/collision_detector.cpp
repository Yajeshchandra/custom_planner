
#include "rrt_star_global_planner/collision_detector.hpp"

namespace rrt_star_global_planner
{

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2D *costmap) : costmap_(costmap)
    {
        if (costmap_ != nullptr)
        {
            resolution_ = costmap_->getResolution();
            origin_x_ = costmap_->getOriginX();
            origin_y_ = costmap_->getOriginY();
            size_x_ = costmap_->getSizeInCellsX();
            size_y_ = costmap_->getSizeInCellsY();
        }
    }

    bool CollisionDetector::isCollision(const std::pair<float, float> &point)
    {
        int mx, my;
        if (!worldToMap(point.first, point.second, mx, my))
        {
            // ROS_INFO("Point is out of map bounds %f %f", point.first, point.second);
            return true;
        }
        int cost = costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        ROS_INFO("Cost of Point %d", cost);
        return cost;
    }

    bool CollisionDetector::isCollision(const Node &node)
    {
        return isCollision(std::make_pair(node.x, node.y));
    }

    bool CollisionDetector::isCoordinateInBound(int mx, int my)
    {
        return mx >= 0 && mx < size_x_ && my >= 0 && my < size_y_;
    }

    bool CollisionDetector::isThereObstacleBetween(float x1, float y1, float x2, float y2)
    {
        float distance = euclideanDistance(x1, y1, x2, y2);
        // float dx,dy;
        auto [dx,dy] = getslope(x1, y1, x2, y2);
        int steps = std::floor(distance / resolution_);
        for (int i = 0; i < steps; i++)
        {
            if (isCollision(std::make_pair(x1 + i * dx, y1 + i * dy)))
            {
                return true;
            }
        }
        return false;
    }
    bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<float, float> &point)
    {
        return isThereObstacleBetween(node.x, node.y, point.first, point.second);
    }

    bool CollisionDetector::isThereObstacleBetween(const Node &node1, const Node &node2)
    {
        return isThereObstacleBetween(node1.x, node1.y, node2.x, node2.y);
    }


    bool CollisionDetector::worldToMap(float wx, float wy, int &mx, int &my)
    {
        if (wx < origin_x_ || wy < origin_y_)
        {
            return false;
        }
        mx = int((wx - origin_x_) / resolution_);
        my = int((wy - origin_y_) / resolution_);
        return mx < size_x_ && my < size_y_;
    }

    float euclideanDistance(Node node1, Node node2)
    {
        return euclideanDistance(node1.x, node1.y, node2.x, node2.y);
    }

    float euclideanDistance(float x1, float y1, float x2, float y2)
    {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }

    std::pair<float,float> getslope(float x1, float y1, float x2, float y2)
    {
        float theta = atan2(y1 - y2, x1 - x2);
        float dx = cos(theta);
        float dy = sin(theta);
        return std::make_pair(dx,dy);
    }

    bool CollisionDetector::isStartAndGoalValid(const Node &start, const Node &goal)
    {
        return !isCollision(start) && !isCollision(goal);
    }

    bool CollisionDetector::isStartAndGoalValid(const std::pair<float, float> &start, const std::pair<float, float> &goal)
    {
        return !isCollision(start) && !isCollision(goal);
    }

} // namespace rrt_star_global_planner
