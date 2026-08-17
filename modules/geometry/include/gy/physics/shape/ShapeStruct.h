/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::geometry {

struct SolidGeometryProperties
{
    math::Real volume{math::kZero};
    math::Vector3 centroid{math::Vector3::Zero()};

    // Unit-density inertia tensor about the geometric centroid.
    math::Matrix3 unitDensityInertiaAtCentroid{math::Matrix3::Zero()};
};

} // namespace gy::physics::geometry
