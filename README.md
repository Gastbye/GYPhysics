# Physics Engine

A general-purpose physics engine.

## Current structure

```text
GyPhysics/
├── CMakeLists.txt
├── cmake/
│   └── GyPhysicsWarnings.cmake
├── modules/
│   ├── math/
│   │   ├── include/gy/physics/math/
│   │   │   ├── MathTypes.h
│   │   │   ├── MathOperators.h
│   │   │   └── QuaternionOperators.h
│   │   └── tests/
│   │       ├── MathTypesTest.cpp
│   │       ├── MathOperatorsTest.cpp
│   │       └── QuaternionOperatorsTest.cpp
│   ├── geometry/
│   │   ├── include/gy/physics/geometry/
│   │   │   ├── Shape.h
│   │   │   ├── ShapeStruct.h
│   │   │   ├── ShapeRegistry.h
│   │   │   ├── SphereShape.h
│   │   │   ├── TriMeshShape.h
│   │   │   └── TriMeshData.h
│   │   ├── src/
│   │   │   ├── ShapeRegistry.cpp
│   │   │   └── TriMeshData.cpp
│   │   └── tests/
│   │       ├── GeometryTest.cpp
│   │       ├── ShapeRegistryTest.cpp
│   │       └── SolidGeometryPropertiesTest.cpp
│   ├── material/
│   │   ├── include/gy/physics/material/
│   │   │   ├── Material.h
│   │   │   ├── MaterialStruct.h
│   │   │   └── MaterialRegistry.h
│   │   ├── src/
│   │   │   ├── Material.cpp
│   │   │   └── MaterialRegistry.cpp
│   │   └── tests/
│   │       ├── MaterialTest.cpp
│   │       └── MaterialRegistryTest.cpp
│   ├── mechanics/
│   │   └── include/gy/physics/mechanics/Mechanics.h
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
| Geometry | `GyPhysics::Geometry` | `gy::physics::geometry` |
| Material | `GyPhysics::Material` | `gy::physics::material` |
| Mechanics | `GyPhysics::Mechanics` | `gy::physics::mechanics` |
| Rigid | `GyPhysics::Rigid` | `gy::physics::rigid` |

`RigidBody` do not use `Eigen::Vector3d` or `Eigen::Vector3f`, use：

```cpp
gy::physics::math::Real
gy::physics::math::Vector3
gy::physics::math::Matrix3
gy::physics::math::Quaternion
```

Windows Compile and Install:
Download MSYS2, and open MSYS2 UCRT64.
Install the latest packages in MSYS2 UCRT64. If it suggests close terminal, close and reopen it:
```bash
pacman -Syu
```

Install the required packages:
```bash
pacman -S --needed \
    mingw-w64-ucrt-x86_64-toolchain \
    mingw-w64-ucrt-x86_64-cmake \
    mingw-w64-ucrt-x86_64-ninja \
    mingw-w64-ucrt-x86_64-eigen3 \
    mingw-w64-ucrt-x86_64-clang-tools-extra
```

Suggest use Tsinghua mirror:
```bash
sed -i "s#https\?://mirror.msys2.org/#https://mirrors.tuna.tsinghua.edu.cn/msys2/#g" /etc/pacman.d/mirrorlist*
grep -n "tuna" /etc/pacman.d/mirrorlist*
pacman -Syyu
pacman -Syu
```
