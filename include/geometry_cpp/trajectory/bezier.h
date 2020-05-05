#pragma once

#include <array>
#include <cmath>
#include <utility>
#include <algorithm>

namespace geometry::trajectory
{

// TODO: Add (partial-)constexpr support.

// TODO: Add concepts support.
// #include <concepts>
// template<typename T>
// concept Vector = requires(T a, T b)
// {
//     { a + b } -> std::same_as<T>;
//      etc..
// };

static constexpr unsigned int factorial(unsigned int n)
{
    return (n == 1 || n == 0) ? 1 : n * factorial(n - 1);
}

template<typename PointType, std::size_t degree>
class Bezier
{
    static constexpr std::size_t size = degree + 1;

private:
    const double duration_ = 1;
    const std::array<PointType, size> points_;

public:
    explicit Bezier(std::array<PointType, size> points) : points_(points)
    {
    }

    Bezier(double duration, std::array<PointType, size> points)
        : duration_(duration)
        , points_(points)
    {
        // TODO: Assert: duration_ > 0
    }

    double duration() const { return duration_; }

    PointType pos(double t) const { return calc_value(t); }
    PointType vel(double t) const { return get_derivative().pos(t); }
    PointType acc(double t) const { return get_derivative().vel(t); }

    Bezier<PointType, degree-1> get_derivative() const
    {
        std::array<PointType, size-1> derivative_points;
        for (std::size_t i = 0; i < size-1; ++i)
        {
            derivative_points[i] = (points_[i+1] - points_[i]) * degree / duration_;
        }
        return Bezier<PointType, degree-1>(derivative_points);
    }

    Bezier<PointType, degree> get_reversed() const
    {
        std::array<PointType, size> reverse_points = points_;
        std::reverse(std::begin(reverse_points), std::end(reverse_points));
        return Bezier{reverse_points};
    }

private:
    PointType calc_value(double t) const
    {
        // TODO: Use De Casteljau's algorithm for better numerical stability at higher degrees?
        PointType result = PointType::Zero();
        for (const auto& coeff : calc_coeffs())
        {
            result = result*t + coeff;
        }
        return result;
    }

    std::array<PointType, size> calc_coeffs() const
    {
        std::array<PointType, size> coeffs;
        for (unsigned int j = 0; j <= degree; ++j)
        {
            PointType cj = PointType::Zero();
            for (unsigned int i = 0; i <= j; ++i)
            {
                cj += std::pow(-1, i+j) * points_[i] / (factorial(i) * factorial(j-i));
            }
            cj *= factorial(degree) / factorial(degree - j);
            cj /= std::pow(duration_, j);
            coeffs[j] = cj;
        }
        std::reverse(std::begin(coeffs), std::end(coeffs));
        return coeffs;
    }

};

/// Split Bézier-curve into two new curves at time t.
template<typename PointType, std::size_t degree>
std::pair<Bezier<PointType, degree>, Bezier<PointType, degree>> split(Bezier<PointType, degree> bezier, double t);

}