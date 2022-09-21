#pragma once

#include "camera.h"
#include "common.h"
#include "dynamic_renderer.h"
#include "input.h"

namespace muli
{
class Game;

class Demo
{
public:
    Demo()
    {
        // simulationDeltaTime = 1.0f / Window::Get().GetRefreshRate();
        simulationDeltaTime = 1.0f / 144.0f;
        settings.VALID_REGION.min.y = -20.0f;

        world = new World(settings);

        camera.scale.Set(1.0f, 1.0f);
        camera.rotation = 0.0f;
        camera.position.Set(0.0f, 3.6f);
    }

    virtual ~Demo()
    {
        delete world;
        world = nullptr;
    }

    virtual void UpdateInput(Game& game);
    virtual void Step(Game& game);
    virtual void UpdateUI(Game& game){};

    virtual void Render(DynamicRenderer& dRenderer) {}

    World& GetWorld()
    {
        return *world;
    }

    WorldSettings& GetWorldSettings()
    {
        return settings;
    }

    Camera& GetCamera()
    {
        return camera;
    }

    RigidBody* GetTarget()
    {
        return target;
    }

protected:
    WorldSettings settings;
    World* world;
    Camera camera;

    float simulationDeltaTime;

    GrabJoint* gj = nullptr;
    RigidBody* target = nullptr;
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