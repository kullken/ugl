#pragma once

#include <cassert>
#include <cmath>
#include <array>
#include <utility>
#include <algorithm>

#include "ugl/math/vector.h"

#include "ugl/trajectory/trajectory.h"

namespace ugl::trajectory
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

// template<std::size_t degree>
template<int degree>
class Bezier : public LinearTrajectory
{
    static constexpr std::size_t size = degree + 1;

private:
    const double duration_ = 1;
    const std::array<ugl::Vector3, size> points_;

public:
    explicit Bezier(std::array<ugl::Vector3, size> points) : points_(points)
    {
    }

    Bezier(double duration, std::array<ugl::Vector3, size> points)
        : duration_(duration)
        , points_(points)
    {
        assert(("Duration must be positive.", duration > 0));
    }

    double duration() const override { return duration_; }

    ugl::Vector3 pos(double t) const override { return calc_value(t); }
    ugl::Vector3 vel(double t) const override { return get_derivative().pos(t); }
    ugl::Vector3 acc(double t) const override { return get_derivative().vel(t); }

    Bezier<degree-1> get_derivative() const
    {
        std::array<ugl::Vector3, size-1> derivative_points;
        for (std::size_t i = 0; i < size-1; ++i)
        {
            derivative_points[i] = (points_[i+1] - points_[i]) * degree / duration_;
        }
        return Bezier<degree-1>(derivative_points);
    }

    Bezier<degree> get_reversed() const
    {
        std::array<ugl::Vector3, size> reverse_points = points_;
        std::reverse(std::begin(reverse_points), std::end(reverse_points));
        return Bezier{reverse_points};
    }

private:
    ugl::Vector3 calc_value(double t) const
    {
        // TODO?: Use De Casteljau's algorithm for better numerical stability at higher degrees?
        ugl::Vector3 result = ugl::Vector3::Zero();
        for (const auto& coeff : calc_coeffs())
        {
            result = result*t + coeff;
        }
        return result;
    }

    std::array<ugl::Vector3, size> calc_coeffs() const
    {
        std::array<ugl::Vector3, size> coeffs;
        for (unsigned int j = 0; j <= degree; ++j)
        {
            ugl::Vector3 cj = ugl::Vector3::Zero();
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

template<>
ugl::Vector3 Bezier<0>::vel([[maybe_unused]] double t) const { return ugl::Vector3::Zero(); }

template<>
ugl::Vector3 Bezier<0>::acc([[maybe_unused]] double t) const { return ugl::Vector3::Zero(); }

template<>
ugl::Vector3 Bezier<1>::acc([[maybe_unused]] double t) const { return ugl::Vector3::Zero(); }

/// Split Bézier-curve into two new curves at time t.
template<std::size_t degree>
std::pair<Bezier<degree>, Bezier<degree>> split(Bezier<degree> bezier, double t);

/// Creates a Bézier curve from a duration and position, velocity and acceleration at start and end.
Bezier<5> createPenticBezier(double duration, ugl::Vector3 pos0, ugl::Vector3 vel0, ugl::Vector3 acc0, ugl::Vector3 pos1, ugl::Vector3 vel1, ugl::Vector3 acc1)
{
    const ugl::Vector3 p0 = pos0;
    const ugl::Vector3 p1 = duration/5 * vel0 + p0;
    const ugl::Vector3 p2 = duration*duration/20 * acc0 + 2*p1 - p0;

    const ugl::Vector3 p5 = pos1;
    const ugl::Vector3 p4 = -duration/5 * vel1 + p5;
    const ugl::Vector3 p3 = duration*duration/20 * acc1 + 2*p4 - p5;

    return Bezier<5>{duration, {p0, p1, p2, p3, p4, p5}};
}

}
