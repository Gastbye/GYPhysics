#include <iostream>

#include "gy/physics/rigid/RigidBody.h"

int main()
{
    namespace math = gy::physics::math;
    namespace rigid = gy::physics::rigid;

    rigid::RigidBodyDesc desc;
    desc.massProperties.mass = static_cast<math::Real>(2.5);
    desc.initialState.position = math::Vector3(
        static_cast<math::Real>(1),
        static_cast<math::Real>(2),
        static_cast<math::Real>(3)
    );

    const rigid::RigidBody body(desc);

    std::cout << "GyPhysics Day 1 example\n"
              << "Real type: " << math::realTypeName() << '\n'
              << "Mass: " << body.mass() << '\n'
              << "Position: " << body.state().position.transpose() << '\n';

    return 0;
}
