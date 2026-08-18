/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::mechanics {

struct MassProperties
{
    math::Real mass{math::kOne};

    // Center of mass in the body's reference local coordinate system.
    math::Vector3 centerOfMassLocalPosition{
        math::Vector3::Zero()
    };

    // Inertia tensor about the center of mass, expressed in body-local axes.
    math::Matrix3 inertiaTensorLocalAtCenterOfMass{
        math::Matrix3::Identity()
    };
};

} // namespace gy::physics::mechanics
