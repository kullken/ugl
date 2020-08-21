#ifndef UGL_RANDOM_SEED_H
#define UGL_RANDOM_SEED_H

#include <random>

namespace ugl::random
{

auto get_seed() -> std::mt19937::result_type;

void set_seed(std::mt19937::result_type seed);

} // namespace ugl::random

#endif // UGL_RANDOM_SEED_H
