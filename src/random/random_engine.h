#pragma once

#include <random>

namespace ugl::random::internal
{

extern std::mt19937::result_type rng_seed;
extern std::mt19937 rng;

} // namespace ugl::random::internal
