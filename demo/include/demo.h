#pragma once

#include "camera.h"
#include "common.h"
#include "input.h"
#include "options.h"

namespace muli
{

class Game;
struct DebugOptions;

class Demo
{
public:
    Demo(Game& _game);
    virtual ~Demo()
    {
        delete world;
        world = nullptr;
    }

    virtual void UpdateInput();
    virtual void Step();
    virtual void UpdateUI() {}
    virtual void Render() {}

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

    RigidBody* GetTargetBody()
    {
        return targetBody;
    }

    Collider* GetTargetCollider()
    {
        return targetCollider;
    }

protected:
    Game& game;
    DebugOptions& options;

    Camera camera;
    WorldSettings settings;
    World* world;

    float dt;
    Vec2 cursorPos;
    std::vector<Collider*> qr;
    RigidBody* targetBody = nullptr;
    Collider* targetCollider = nullptr;
    GrabJoint* gj = nullptr;

    void ComputeProperty();
    void EnableKeyboardShortcut();
    void EnableBodyCreate();
    bool EnablePolygonCreate();
    void EnableBodyRemove();
    bool EnableAddForce();
    bool EnableBodyGrab();
    void EnableCameraControl();
};

typedef Demo* DemoCreateFunction(Game& game);

struct DemoFrame
{
    const char* name;
    DemoCreateFunction* createFunction;
};

#define MAX_DEMOS 100
extern uint32 demo_count;
extern DemoFrame demos[MAX_DEMOS];

} // namespace muli