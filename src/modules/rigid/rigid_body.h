/*
** (C) Copyright 2026, Guangyu, All Rights Reserved
*/

#pragma once
#include <iostream>

namespace gy::physics::rigid {

using namespace Eigen;

class RigidBody 
{
public:
    RigidBody() = default;
    ~RigidBody() = default;

    void update(float deltaTime) {
        // Update the rigid body's state based on physics calculations
        std::cout << "Updating RigidBody with deltaTime: " << deltaTime << std::endl;
    }

    double mass;

    Vector3d position;
    Quaterniond orientation;

    Vector3d linearVelocity;
    Vector3d angularVelocity;
    
    Vector3d force;
    Vector3d torque;

    Matrix3d inertiaTensor;
    Matrix3d inverseInertiaTensor;
}


    

}
