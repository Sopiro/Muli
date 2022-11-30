# Muli
[![CMake Build](https://github.com/Sopiro/Muli/actions/workflows/cmake_build.yml/badge.svg)](https://github.com/Sopiro/Muli/actions/workflows/cmake_build.yml)

2D Rigidbody physics engine

## Example
```c++
#include <iostream>
#include "muli/muli.h"

using namespace muli;

int main()
{
    WorldSettings settings; // Define simulation settings
    World world(settings); // Create physics world
  
    RigidBody* box = world.CreateBox(1.0f); // Create box
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
