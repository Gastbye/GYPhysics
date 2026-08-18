/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::mechanics {

struct MassProperties
{
    math::Real mass{math::kOne};

    // Center of mass of the body: local coordinates.
    math::Vector3 centerOfMassLocalPosition{
        math::Vector3::Zero()
    };

    // Inertia tensor at the center of mass of the body: local coordinates.
    math::Matrix3 inertiaTensorLocalAtCenterOfMass{
        math::Matrix3::Identity()
    };
};

} // namespace gy::physics::mechanics