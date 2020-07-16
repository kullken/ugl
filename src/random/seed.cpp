#include "ugl/random/seed.h"

#include <random>

#include "random_engine.h"

namespace ugl::random
{

auto get_seed() -> std::mt19937::result_type
{
    return internal::rng_seed;
}

void set_seed(std::mt19937::result_type seed)
{
    internal::rng_seed = seed;
    internal::rng.seed(seed);
}

} // namespace ugl::random
