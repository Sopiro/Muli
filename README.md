# Muli

![logo](.github/logo.gif)

[![CMake Build](https://github.com/Sopiro/Muli/actions/workflows/cmake_build.yml/badge.svg)](https://github.com/Sopiro/Muli/actions/workflows/cmake_build.yml)

2D Rigidbody physics engine

## Features  

### Collision  
  - Shapes: circle, capsule and convex polygon
  - Support rounded polygon
  - Multiple shapes attached to single body
  - Dynamic, static and kinematic bodies
  - Collision filtering
  - Dynamic AABB tree broadphase
  - Dynamic tree accelerated ray casting, area querying
  - Easy-to-use collision detection and distance funtions
  
### Physics Simulation
  - Constrained dynamics
  - Efficient temporal coherent contact management from box2d
  - Constraint islanding and sleeping
  - Stable stacking with 2-contact LCP solver (Block solver)
  - Decoupled position correction iteration
  - Contact callbacks: begin, touching, end, destory
  - Physics material: friction, restitution and surface speed
  - Various joints: angle, distance, grab, line, motor, prismatic, pulley, revolute, weld
  - Tunable joint parameters  
  
### ETC
  - 40+ Demos
  - OpenGL based demo framework
  - Cross platform C++17 library
  - Intuitive and straightforward API
  - Utilize efficient custom memory allocators
  
## Example
```c++
#include "muli/muli.h"

using namespace muli;

int main()
{
    WorldSettings settings; // Define simulation settings
    World world(settings); // Create physics world
  
    RigidBody* box = world.CreateBox(1.0f); // Create a box
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

## Building and running
- Install [CMake](https://cmake.org/install/)
- Ensure CMake is in the system `PATH`
- Clone the repository `git clone --recursive https://github.com/Sopiro/Muli`
- Run CMake build script depend on your system
  - Visual Studio: Run `build.bat`
  - Otherwise: Run `build.sh`
- You can find the executable demo in the `build/bin`

## Todo
- Implement position solve step for joints
- Continuous simulation  

## References

Here are great resources to learn building a physics engine!

- https://box2d.org/publications/
- https://allenchou.net/game-physics-series/
- https://dyn4j.org/blog/
- https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics
