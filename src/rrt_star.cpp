#include "rrt_star_global_planner/rrt_star.hpp"

namespace rrt_star_global_planner
{

    RRTStar::RRTStar(const std::pair<float, float> &start_point,
                     const std::pair<float, float> &goal_point,
                     costmap_2d::Costmap2D *costmap,
                     double goal_tolerance,
                     double radius,
                     double epsilon,
                     unsigned int max_num_nodes,
                     unsigned int min_num_nodes,
                     float map_width,
                     float map_height) : start_point_(start_point),
                                         goal_point_(goal_point),
                                         costmap_(costmap),
                                         goal_tolerance_(goal_tolerance),
                                         radius_(radius),
                                         epsilon_(epsilon),
                                         max_num_nodes_(max_num_nodes),
                                         min_num_nodes_(min_num_nodes),
                                         map_width_(map_width),
                                         map_height_(map_height),
                                         cd_(costmap)
    {
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();

        nodes_.reserve(max_num_nodes_);
        coordinate_generator_.setXRange(origin_x_, origin_x_ + map_width_);
        coordinate_generator_.setYRange(origin_y_, origin_y_ + map_height_);
    }

    bool RRTStar::pathPlanning(std::list<std::pair<float, float>> &path)
    {
        goal_reached_ = false;

        if (!(cd_.isStartAndGoalValid(start_point_, goal_point_)))
        {
            // if (cd_.isCollision(start_point_)) ROS_ERROR("Start point is in collision");
            // if (cd_.isCollision(goal_point_)) ROS_ERROR("Goal point is in collision");
            ROS_ERROR("Start and Goal points are in collision");
            return false;
        }

        // Start Node
        createNewNode(start_point_.first, start_point_.second, -1);

        std::pair<float, float> p_rand;
        std::pair<float, float> p_new;

        Node node_nearest;

        bool found_next;
        while (nodes_.size() < max_num_nodes_)
        {
            found_next = false;
            while (!found_next)
            {
                p_rand = coordinate_generator_.generateCoordinate();                        // random point in the free space
                node_nearest = nodes_[getNearestNodeId(p_rand)];                            // nearest node of the random point
                p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate
                if (!cd_.isThereObstacleBetween(node_nearest, p_new))
                {
                    found_next = true;
                    createNewNode(p_new.first, p_new.second, node_nearest.node_id);
                }
            }

            if (!goal_reached_)
            {
                if (isGoalReached(p_new))
                {
                    goal_reached_ = true;
                    goal_node_ = nodes_.back();
                }
            }

            if (goal_reached_ && nodes_.size() > min_num_nodes_)
            {
                computeFinalPath(path);
                return true;
            }
        }
        return false;
    }

    int RRTStar::getNearestNodeId(const std::pair<float, float> &point)
    {
        float dist_nearest, dist;
        Node node_nearest = nodes_[0];
        for (int i = 1; i < nodes_.size(); ++i)
        {
            dist_nearest = euclideanDistance(node_nearest.x, node_nearest.y, point.first, point.second);
            dist = euclideanDistance(nodes_[i].x, nodes_[i].y, point.first, point.second);
            if (dist < dist_nearest)
                node_nearest = nodes_[i];
        }

        return node_nearest.node_id;
    }

    void RRTStar::createNewNode(float x, float y, int node_nearest_id)
    {
        Node new_node(x, y, node_count_, node_nearest_id);
        nodes_.emplace_back(new_node);

        if (node_nearest_id != -1)
        {
            // Optimize
            chooseParent(node_nearest_id);
            rewire();
        }

        node_count_++;
    }

    void RRTStar::chooseParent(int node_nearest_id)
    {
        float cost_new_node;
        float cost_other_parent;
        float nodes_dist;

        Node parent_node = nodes_[node_nearest_id];

        Node &new_node = nodes_.back();

        for (const auto &node : nodes_)
        {
            if (node.node_id == new_node.node_id)
                continue;
            // distance between node and new_node
            nodes_dist = euclideanDistance(node, new_node);

            if (nodes_dist < radius_)
            {
                // current cost of new_node
                cost_new_node = parent_node.cost + euclideanDistance(parent_node, new_node);

                // cost if the parent is node
                cost_other_parent = node.cost + nodes_dist;

                if (cost_other_parent < cost_new_node)
                {
                    if (!cd_.isThereObstacleBetween(node, new_node))
                    {
                        parent_node = node;
                    }
                }
            }
        }

        // Update new_node cost and its new parent
        new_node.cost = parent_node.cost + euclideanDistance(parent_node, new_node);
        new_node.parent_id = parent_node.node_id;
    }

    void RRTStar::rewire()
    {
        float nodes_dist;
        float cost_node;

        Node new_node = nodes_.back();

        for (auto &node : nodes_)
        {
            // distance between node and new_node
            nodes_dist = euclideanDistance(node.x, node.y, new_node.x, new_node.y);

            // check if node is already the parent and if node is near for optimization
            if (node != nodes_[new_node.parent_id] && nodes_dist < radius_)
            {
                // cost if the parent of node is new_node
                cost_node = new_node.cost + euclideanDistance(node.x, node.y, new_node.x, new_node.y);

                if (cost_node < node.cost && !cd_.isThereObstacleBetween(node, new_node))
                {
                    // update the new parent of node and its new cost
                    node.parent_id = new_node.node_id;
                    node.cost = cost_node;
                }
            }
        }
    }

    // TODO(Rafael) improve parameters name
    std::pair<float, float> RRTStar::steer(float x1, float y1, float x2, float y2)
    {
        std::pair<float, float> p_new;
        float dist = euclideanDistance(x1, y1, x2, y2);
        if (dist < epsilon_)
        {
            p_new.first = x1;
            p_new.second = y1;
            return p_new;
        }
        else
        {
            // float dx, dy;
            auto[dx, dy] = getslope(x1, y1, x2, y2);
            p_new.first = x1 + epsilon_ * dx;
            p_new.second = y1 + epsilon_ * dy;
            return p_new;
        }
    }

    std::vector<Node> RRTStar::getNodes() const
    {
        return nodes_;
    }

    void RRTStar::computeFinalPath(std::list<std::pair<float, float>> &path)
    {
        path.clear();

        // Compute the path from the goal to the start
        Node current_node = goal_node_;

        // Final Path
        std::pair<float, float> point;

        do
        {
            point.first = current_node.x;
            point.second = current_node.y;
            path.push_front(point);

            // update the current node
            current_node = nodes_[current_node.parent_id];
        } while (current_node.parent_id != -1);
    }

    bool RRTStar::isGoalReached(const std::pair<float, float> &p_new)
    {
        return (euclideanDistance(p_new.first,
                                    p_new.second,
                                    goal_point_.first,
                                    goal_point_.second) < goal_tolerance_)
                   ? true
                   : false;
    }

} // namespace rrt_star_global_planner