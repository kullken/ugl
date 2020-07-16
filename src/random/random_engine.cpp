#include "random_engine.h"

#include <random>

namespace ugl::random::internal
{

std::mt19937::result_type rng_seed = std::mt19937::default_seed;
std::mt19937 rng{};

} // namespace ugl::random::internal
