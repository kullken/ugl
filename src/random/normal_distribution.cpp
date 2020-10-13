#include "ugl/random/normal_distribution.h"

#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>

#include "random_engine.h"

namespace ugl::random
{

ugl::MatrixD create_transform(const ugl::MatrixD& covariance)
{
    Eigen::SelfAdjointEigenSolver<ugl::MatrixD> eigen_solver(covariance);
    return eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
}

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
