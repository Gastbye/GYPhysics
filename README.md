# Physics Engine

A general-purpose physics engine.

## current structure

```text
GyPhysics/
├── CMakeLists.txt
├── cmake/
│   └── GyPhysicsWarnings.cmake
├── modules/
│   ├── math/
│   │   ├── include/gy/physics/math/MathTypes.h
│   │   └── tests/MathTypesTest.cpp
│   └── rigid/
│       ├── include/gy/physics/rigid/RigidBody.h
│       ├── src/RigidBody.cpp
│       └── tests/RigidBodyTest.cpp
└── examples/
    └── rigid/main.cpp
```

## Ubuntu dependencies

```bash
sudo apt update
sudo apt install build-essential cmake ninja-build libeigen3-dev
```

## double precision build and test

```bash
cd /path/GYPhysics
cmake -S . -B build-double -G Ninja -DCMAKE_BUILD_TYPE=Release -DGY_PHYSICS_REAL_TYPE=DOUBLE -DBUILD_TESTING=ON -DGY_PHYSICS_BUILD_EXAMPLES=ON
cmake --build build-double -j "$(nproc)"
ctest --test-dir build-double --output-on-failure
./build-double/examples/rigid/gyphysics_rigid_example
```

## float precision build and test

```bash
cd /path/GYPhysics
cmake -S . -B build-float -G Ninja -DCMAKE_BUILD_TYPE=Release -DGY_PHYSICS_REAL_TYPE=FLOAT -DBUILD_TESTING=ON -DGY_PHYSICS_BUILD_EXAMPLES=ON
cmake --build build-float -j "$(nproc)"
ctest --test-dir build-float --output-on-failure
./build-float/examples/rigid/gyphysics_rigid_example
```

## validate a module separately

Only compile and run Math tests:

```bash
cd /path/GYPhysics
cmake -S . -B build-double -G Ninja -DCMAKE_BUILD_TYPE=Release -DGY_PHYSICS_REAL_TYPE=DOUBLE -DBUILD_TESTING=ON -DGY_PHYSICS_BUILD_EXAMPLES=ON
cmake --build build-double --target gyphysics_math_tests -j "$(nproc)"
ctest --test-dir build-double -R '^gyphysics\.math$' --output-on-failure
```

Only compile and run Rigid tests:

```bash
cmake -S . -B build-double -G Ninja -DCMAKE_BUILD_TYPE=Release -DGY_PHYSICS_REAL_TYPE=DOUBLE -DBUILD_TESTING=ON -DGY_PHYSICS_BUILD_EXAMPLES=ON
cmake --build build-double --target gyphysics_rigid -j "$(nproc)"
cmake --build build-double --target gyphysics_rigid_tests -j "$(nproc)"
ctest --test-dir build-double -R '^gyphysics\.rigid$' --output-on-failure
```

Only compile Rigid library:

```bash
cmake --build build-double --target gyphysics_rigid
```

## CMake targets and namespaces

| 模块 | CMake 目标 | C++ 命名空间 |
|---|---|---|
| Math | `GyPhysics::Math` | `gy::physics::math` |
| Rigid | `GyPhysics::Rigid` | `gy::physics::rigid` |

`RigidBody` 不直接使用 `Eigen::Vector3d` 或 `Eigen::Vector3f`，而统一使用：

```cpp
gy::physics::math::Real
gy::physics::math::Vector3
gy::physics::math::Matrix3
gy::physics::math::Quaternion
```

因此切换 `GY_PHYSICS_REAL_TYPE` 时，整个项目的标量和 Eigen 固定尺寸类型会一起
切换。不要在同一个进程里链接分别用 FLOAT 和 DOUBLE 编译的 GyPhysics 模块。
