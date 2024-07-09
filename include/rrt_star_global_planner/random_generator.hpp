#ifndef RRT_STAR_GLOBAL_PLANNER_RANDOM_FLOAT_GENERATOR_HPP_ // NOLfloat
#define RRT_STAR_GLOBAL_PLANNER_RANDOM_FLOAT_GENERATOR_HPP_

#include <random>
#include <cfloat> // DBL_MAX

namespace rrt_star_global_planner
{

    class RandomFloatGenerator
    {
    private:
        std::random_device rd_;
        float min_value_{-1.0};
        float max_value_{1.0};

    public:
        RandomFloatGenerator() = default;

        void setRange(float min, float max);

        float generate();
    };

    class CoordinateGenerator : public RandomFloatGenerator
    {
    private:
        RandomFloatGenerator x_generator_;
        RandomFloatGenerator y_generator_;
        float x_min_{-1.0};
        float x_max_{1.0};
        float y_min_{-1.0};
        float y_max_{1.0};

    public:
        CoordinateGenerator() = default;
        CoordinateGenerator(float x_min, float x_max, float y_min, float y_max);

        std::pair<float, float> generateCoordinate();

        void setXRange(float min, float max);

        void setYRange(float min, float max);
    };

} // namespace rrt_star_global_planner

#endif // RRT_STAR_GLOBAL_PLANNER_RANDOM_float_GENERATOR_HPP_  // NOLfloat