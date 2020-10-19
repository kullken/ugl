#include "ugl/random/uniform_distribution.h"

#include <random>

#include "random_engine.h"

namespace ugl::random
{

UniformDistribution<1>::UniformDistribution(double upper_bound)
    : upper_bound_(upper_bound)
{
}

UniformDistribution<1>::UniformDistribution(double lower_bound, double upper_bound)
    : lower_bound_(lower_bound)
    , upper_bound_(upper_bound)
{
}

double UniformDistribution<1>::sample() const
{
    return sample(lower_bound_, upper_bound_);
}

double UniformDistribution<1>::sample(double upper_bound)
{
    return sample(0.0, upper_bound);
}

double UniformDistribution<1>::sample(double lower_bound, double upper_bound)
{
    std::uniform_real_distribution distribution{lower_bound, upper_bound};
    return distribution(internal::rng);
}

} // namespace ugl::random
