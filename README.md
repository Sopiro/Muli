![logo](.github/logo.gif)

# Muli

[![Build library](https://github.com/Sopiro/Muli/actions/workflows/cmake-multi-platform.yml/badge.svg)](https://github.com/Sopiro/Muli/actions/workflows/cmake-multi-platform.yml)

2D Rigid body physics engine.

## Features

### Collision
  - Continuous collision detection (Bilateral advancement by Erin Catto of Box2D)
  - Shapes: circle, capsule, and convex polygon
  - Support for rounded polygons
  - Multiple colliders attached to a single body
  - Dynamic, static, and kinematic bodies
  - Collision filtering
  - Dynamic AABB tree broadphase
  - Raycast, shapecast, and area queries

### Physics Simulation
  - Continuous physics simulation with time of impact solver
  - PGS solver with a separate position correction (PGS NGS)
  - Stable stacking with a 2-contact LCP solver (Block solver)
  - Persistent constraint graph
  - Constraint islanding and sleeping
  - Contact callbacks: begin, touching, end, pre-solve, post-solve, and destroy
  - Physics material: friction, restitution, and surface speed
  - Various joint types: angle, distance, grab, line, motor, prismatic, pulley, revolute, and weld
  - Joints with limits and motors

### Others
  - Cross platform library (C++20)
  - Intuitive and straightforward API design

### Demo
  - 50+ interactive demos
  - OpenGL based cross platform demo framework
  - WebAssembly powered web [demo](https://sopiro.github.io/muli-wasm/)

## Example

```c++
#include "muli/muli.h"

using namespace muli;

int main()
{
    WorldSettings settings;
    World world(settings);

    Body* box = world.CreateBox(1.0f);
    box->SetPosition(0.0f, 5.0f);

    // Run simulation for one second
    float dt = 1.0f / 60.0f;
    for(int i = 0; i < 60; ++i)
    {
        world.Step(dt);
    }

    return 0;
}
```

## Building and running the demo
- Install [CMake](https://cmake.org/install/).
- Ensure CMake is in the system `PATH`.
- Clone the repository: `git clone https://github.com/Sopiro/Muli`
- Run CMake build script for your system:
  - Visual Studio: Run `build.bat`.
  - Other systems: Run `build.sh`.
- You can find the demo executable in `build/bin`.

## Including the library
You can easily include the library using CMake's `FetchContent` module.

```cmake
include(FetchContent)

FetchContent_Declare(
    muli
    GIT_REPOSITORY https://github.com/Sopiro/Muli
)

set(MULI_BUILD_DEMO OFF)
FetchContent_MakeAvailable(muli)
```

## Installing the library

You can install the library using these commands:

```bat
mkdir build
cd build
cmake -DMULI_BUILD_DEMO=OFF ..
cmake --build . --config Release
cmake --install . --prefix "installation-path"
```

Assuming you've added `"installation-path"` to your system `PATH`, you can now integrate the library into your project.

```cmake
find_package(muli REQUIRED)

target_link_libraries(your-project PRIVATE muli::muli)
```

## References
Here are some great resources for learning how to build a physics engine:
- https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics
- https://allenchou.net/game-physics-series/
- https://box2d.org/publications/
- https://www.cs.cmu.edu/~baraff/sigcourse/index.html
- https://dyn4j.org/blog/
