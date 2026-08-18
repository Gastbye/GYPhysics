/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once

#include "gy/physics/math/MathTypes.h"

namespace gy::physics::rigid {

class RigidBody;

void integrateRigidBody(
    RigidBody& body,
    math::Real timeStep);

} // namespace gy::physics::rigid