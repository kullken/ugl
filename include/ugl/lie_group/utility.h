#pragma once

namespace ugl::lie
{

/// Get identity element of Type. 
/// Defaults to Type::Identity() as used in Eigen.
template<typename Type>
Type identity()
{
    return Type::Identity();
}

}
