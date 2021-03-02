#ifndef UGL_LIE_EUCLIDEAN_H
#define UGL_LIE_EUCLIDEAN_H

#include "ugl/math/vector.h"
#include "ugl/math/matrix.h"

#include "ugl/lie_group/common.h"

namespace ugl::lie
{

template<int dim>
class Euclidean
{
public:
    /// @brief Degrees of freedom of the Lie group.
    static constexpr int DoF = dim;

    using VectorType  = ugl::Vector<DoF>;

public:
    Euclidean() = default;

    explicit Euclidean(const ugl::Vector<dim>& vector)
        : vector_(vector)
    {
    }

    const auto& vector() const
    {
        return vector_;
    }

    /// @brief Lie group inverse of an Euclidean vector.
    /// @return The negation of the vector.
    Euclidean inverse() const
    {
        return Euclidean{-vector_};
    }

    Euclidean& operator*=(const Euclidean& rhs)
    {
        vector_ += rhs.vector_;
        return *this;
    }

    /// @brief Lie Group identity element for Euclidean vectors.
    /// @return A zero vector.
    static Euclidean Identity()
    {
        return Euclidean{};
    }

    static Euclidean exp(const VectorType& v)
    {
        return Euclidean{v};
    }

    static VectorType log(const Euclidean& E)
    {
        return VectorType{E.vector_};
    }

    /// @brief Left and right Jacobians for Euclidean vectors are no-ops.
    /// @return An identity matrix.
    static ugl::Matrix<dim,dim> left_jacobian(const VectorType&)
    {
        return ugl::Matrix<dim,dim>::Identity();
    }

    /// @brief Left and right Jacobians for Euclidean vectors are no-ops.
    /// @return An identity matrix.
    static ugl::Matrix<dim,dim> left_jacobian_inv(const VectorType&)
    {
        return ugl::Matrix<dim,dim>::Identity();
    }

    /// @brief Left and right Jacobians for Euclidean vectors are no-ops.
    /// @return An identity matrix.
    static ugl::Matrix<dim,dim> right_jacobian(const VectorType&)
    {
        return ugl::Matrix<dim,dim>::Identity();
    }

    /// @brief Left and right Jacobians for Euclidean vectors are no-ops.
    /// @return An identity matrix.
    static ugl::Matrix<dim,dim> right_jacobian_inv(const VectorType&)
    {
        return ugl::Matrix<dim,dim>::Identity();
    }

private:
    ugl::Vector<dim> vector_ = ugl::Vector<dim>::Zero();
};

/// @brief Group composition of Euclidean vectors.
/// @return The sum of the vectors.
template<int dim>
Euclidean<dim> operator*(Euclidean<dim> lhs, const Euclidean<dim>& rhs)
{
    return lhs *= rhs;
}

} // namespace ugl::lie

#endif // UGL_LIE_EUCLIDEAN_H
