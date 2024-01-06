#ifndef UGL_LIE_COMMON_H
#define UGL_LIE_COMMON_H

namespace ugl::lie
{

template<typename LieGroup>
LieGroup exp(const typename LieGroup::VectorType& tau)
{
    return LieGroup::exp(tau);
}

template<typename LieGroup>
typename LieGroup::VectorType log(const LieGroup& T)
{
    return LieGroup::log(T);
}

/// @brief Right oplus-operator on Lie group.
/// @param X Lie group element
/// @param tau tangent vector at X
/// @return Lie group element: Y = X * Exp(tau)
template<typename LieGroup>
LieGroup oplus(const LieGroup& X, const typename LieGroup::VectorType& tau)
{
    return X * LieGroup::exp(tau);
}

/// @brief Right ominus-operator on Lie group.
/// @param Y Lie group element
/// @param X Lie group element
/// @return Tangent vector at X: tau = Log(X^-1 * Y)
template<typename LieGroup>
typename LieGroup::VectorType ominus(const LieGroup& Y, const LieGroup& X)
{
    return LieGroup::log(X.inverse() * Y);
}

/// @brief Interpolate between two values.
/// @param start Start value of the interpolation
/// @param end End value of the interpolation
/// @param ratio The ratio of interpolation. A ratio outside interval [0,1] will result in
/// extrapolation.
/// @return The interpolated value
template <typename LieGroup>
LieGroup interpolate(const LieGroup &start, const LieGroup &end, double ratio)
{
    return oplus(start, ratio * ominus(end, start));
}

} // namespace ugl::lie

#endif // UGL_LIE_COMMON_H