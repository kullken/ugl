#ifndef UGL_RANDOM_UNIFORM_DISTRIBUTION_H
#define UGL_RANDOM_UNIFORM_DISTRIBUTION_H

#include "ugl/math/vector.h"

namespace ugl::random
{

template<int dim>
class UniformDistribution
{
public:
    using VectorType = ugl::Vector<dim>;

    UniformDistribution() = default;

    explicit
    UniformDistribution(const VectorType& upper_bound)
        : upper_bound_(upper_bound)
    {
    }

    UniformDistribution(const VectorType& lower_bound, const VectorType& upper_bound)
        : lower_bound_(lower_bound)
        , upper_bound_(upper_bound)
    {
    }

    [[nodiscard]]
    const VectorType& lower_bound() const { return lower_bound_; }
    [[nodiscard]]
    const VectorType& upper_bound() const { return upper_bound_; }

    [[nodiscard]]
    VectorType sample() const
    {
        return sample(lower_bound_, upper_bound_);
    }

    [[nodiscard]]
    static VectorType sample(const VectorType& upper_bound)
    {
        return sample(VectorType::Zero(), upper_bound);
    }

    [[nodiscard]]
    static VectorType sample(const VectorType& lower_bound, const VectorType& upper_bound)
    {
        return lower_bound + (upper_bound - lower_bound).asDiagonal() * sample_standard_noise();
    }

private:
    [[nodiscard]]
    static VectorType sample_standard_noise();

private:
    VectorType lower_bound_ = VectorType::Zero();
    VectorType upper_bound_ = VectorType::Ones();
};


template<>
class UniformDistribution<1>
{
public:
    UniformDistribution() = default;

    explicit
    UniformDistribution(double upper_bound);
    UniformDistribution(double lower_bound, double upper_bound);

    [[nodiscard]]
    double lower_bound() const { return lower_bound_; }
    [[nodiscard]]
    double upper_bound() const { return upper_bound_; }

    [[nodiscard]]
    double sample() const;
    [[nodiscard]]
    static double sample(double upper_bound);
    [[nodiscard]]
    static double sample(double lower_bound, double upper_bound);

private:
    double lower_bound_ = 0.0;
    double upper_bound_ = 1.0;
};

template<int dim>
typename UniformDistribution<dim>::VectorType UniformDistribution<dim>::sample_standard_noise()
{
    VectorType sample;
    for (int i = 0; i < dim; ++i)
    {
        sample[i] = UniformDistribution<1>::sample(0, 1);
    }
    return sample;
}

} // namespace ugl::random

#endif // UGL_RANDOM_UNIFORM_DISTRIBUTION_H
