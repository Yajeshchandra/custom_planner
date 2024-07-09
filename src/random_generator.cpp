#include "rrt_star_global_planner/random_generator.hpp"

namespace rrt_star_global_planner
{

    void RandomFloatGenerator::setRange(float min, float max)
    {
        min_value_ = min;
        max_value_ = max;
    }

    float RandomFloatGenerator::generate()
    {
        std::random_device rd_;
        std::mt19937 gen(rd_());
        std::uniform_real_distribution<float> dis(min_value_, std::nextafter(max_value_, DBL_MAX));

        return dis(gen);
    }

    CoordinateGenerator::CoordinateGenerator(float x_min, float x_max, float y_min, float y_max): x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max)
    {
        x_generator_.setRange(x_min_, x_max_);
        y_generator_.setRange(y_min_, y_max_);
    }

    std::pair<float, float> CoordinateGenerator::generateCoordinate()
    {
        float x = x_generator_.generate();
        float y = y_generator_.generate();
        return std::make_pair(x, y);
    }

    void CoordinateGenerator::setXRange(float min, float max)
    {
        x_min_ = min;
        x_max_ = max;
        x_generator_.setRange(min, max);
    }

    void CoordinateGenerator::setYRange(float min, float max)
    {
        y_min_ = min;
        y_max_ = max;
        y_generator_.setRange(min, max);
    }

} // namespace rrt_star_global_planner