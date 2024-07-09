# RRT Star Global Planner

Planner is based on RRT* algorithm. It is a global planner for ROS navigation stack. It is implemented in C++ and is based on the nav_core::BaseGlobalPlanner interface.  It is based on sampling-based algorithms and is capable of planning in high-dimensional spaces. It is a probabilistic roadmap method that efficiently computes an open-loop solution to the problem of steering a point from a start to a goal in a space cluttered with obstacles. The algorithm is based on the RRT algorithm, which is a randomized algorithm that builds a tree of possible states in the space. The RRT* algorithm is an extension of the RRT algorithm that optimizes the tree to minimize the cost of the path.

## Installation

1. Clone the repository into your catkin workspace
2. Build the package using catkin_make

## Usage


1. Set the global planner to rrt_star_global_planner/rrt_star_global_planner in the move_base node
```xml
<param name="base_global_planner" value="rrt_star_global_planner/RRTStarPlanner"/>
```
2. Set the parameters in the rrt_star_global_planner_params.yaml file
3. Add rosparam to the move_base node
```xml
<rosparam file="$(find custom_global_planner)/config/rrt_star_params.yaml" command="load"/>
```

## Simulation

1. Launch the turtlebot3_world.launch file
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
2. Launch the move_base node
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```
3. Set the 2D Nav Goal in RViz

## Tutorial

A tutorial on how to use the package can be found [here]()

## Aknowledgements
Copy of the original repository:

[rafaelbarretorb-rrt-star-global-planner](https://github.com/rafaelbarretorb/rrt_star_global_planner.git)