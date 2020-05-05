#ifndef _GEOMETRY_LIE_GROUP_CORE_H
#define _GEOMETRY_LIE_GROUP_CORE_H

#include "math/types.h"

namespace geometry::lie
{

/// Get identity element of Type. 
/// Defaults to Type::Identity() as used in Eigen.
template<typename Type>
Type identity()
{
    return Type::Identity();
}

}

#endif