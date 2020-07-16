#include "ugl/random/normal_distribution.h"

#include <cmath>
#include <random>

#include "random_engine.h"

namespace ugl::random
{

NormalDistribution<1>::NormalDistribution(double mean, double variance)
    : mean_(mean)
    , variance_(variance)
    , stddev_(std::sqrt(variance))
{
}

double NormalDistribution<1>::sample() const
{
    std::normal_distribution distribution{mean_, stddev_};
    return distribution(internal::rng);
}

double NormalDistribution<1>::sample(double mean, double variance)
{
    std::normal_distribution distribution{mean, std::sqrt(variance)};
    return distribution(internal::rng);
}

} // namespace ugl::random
