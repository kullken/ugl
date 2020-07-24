#pragma once

#include <Eigen/Eigenvalues>

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

namespace ugl::random
{

template<int size>
class NormalDistribution
{
public:
    using VectorType = ugl::Vector<size>;
    using MatrixType = ugl::Matrix<size, size>;

    NormalDistribution() = default;

    explicit NormalDistribution(const MatrixType& covar)
        : covar_(covar)
        , transform_(create_transform(covar))
    {
    }

    NormalDistribution(const VectorType& mean, const MatrixType& covar)
        : mean_(mean)
        , covar_(covar)
        , transform_(create_transform(covar))
    {
    }

    const VectorType& mean() const { return mean_; }
    const MatrixType& covar() const { return covar_; }

    VectorType sample() const
    {
        return mean_ + transform_ * sample_white_noise();
    }

    static VectorType sample(const VectorType& mean, const MatrixType& covar)
    {
        return mean + create_transform(covar) * sample_white_noise();
    }

private:
    static VectorType sample_white_noise();

    static MatrixType create_transform(const MatrixType& covar)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        return eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

private:
    VectorType mean_ = VectorType::Zero();
    MatrixType covar_ = MatrixType::Identity();
    MatrixType transform_ = MatrixType::Identity();
};


template<>
class NormalDistribution<1>
{
public:
    NormalDistribution() = default;
    NormalDistribution(double mean, double variance);

    double mean() const { return mean_; }
    double stddev() const { return stddev_; }
    double variance() const { return variance_; }

    double sample() const;
    static double sample(double mean, double variance);

private:
    const double mean_ = 0.0;
    const double variance_ = 1.0;
    const double stddev_ = 1.0;
};

template<int size>
typename NormalDistribution<size>::VectorType NormalDistribution<size>::sample_white_noise()
{
    VectorType sample;
    for (int i = 0; i < size; ++i)
    {
        sample[i] = NormalDistribution<1>::sample(0, 1);
    }
    return sample;
}

} // namespace ugl::random
