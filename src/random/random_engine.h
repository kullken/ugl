#ifndef UGL_RANDOM_RANDOM_ENGINE_H
#define UGL_RANDOM_RANDOM_ENGINE_H

#include <random>

namespace ugl::random::internal
{

extern std::mt19937::result_type rng_seed;
extern std::mt19937 rng;

} // namespace ugl::random::internal

#endif // UGL_RANDOM_RANDOM_ENGINE_H
