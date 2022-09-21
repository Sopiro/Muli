#pragma once

#include "common.h"
#include "dynamic_renderer.h"
#include "input.h"

namespace muli
{

class Demo
{
public:
    Demo()
    {
        // simulationDeltaTime = 1.0f / Window::Get().GetRefreshRate();
        simulationDeltaTime = 1.0f / 144.0f;
        settings.VALID_REGION.min.y = -20.0f;

        world = new World(settings);
    }

    virtual ~Demo()
    {
        delete world;
        world = nullptr;
    }

    virtual void Step()
    {
        world->Step(simulationDeltaTime);
    }

    virtual void Render(DynamicRenderer& dRenderer) {}

    World& GetWorld()
    {
        return *world;
    }

    WorldSettings& GetWorldSettings()
    {
        return settings;
    }

protected:
    WorldSettings settings;
    World* world;

    float simulationDeltaTime;
};

typedef Demo* DemoCreateFunction();

struct DemoFrame
{
    const char* name;
    DemoCreateFunction* createFunction;
};

#define MAX_DEMOS 100
extern uint32 demo_count;
extern DemoFrame demos[MAX_DEMOS];

} // namespace muli