#pragma once

#include "camera.h"
#include "common.h"
#include "input.h"
#include "muli/random.h"
#include "options.h"

namespace muli
{

class Game;
class Renderer;
struct DebugOptions;

class Demo : public JointDestroyCallback
{
public:
    Demo(Game& game);
    virtual ~Demo();

    virtual void UpdateInput();
    virtual void Step();
    virtual void UpdateUI() {}
    virtual void Render() {}

    // from JointDestroyCallback
    virtual void OnJointDestroy(Joint* me) override
    {
        cursorJoint = nullptr;
    }

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
    void FindTargetBody();
    void EnableKeyboardShortcut();
    void EnableBodyCreate();
    bool EnablePolygonCreate();
    void EnableBodyRemove();
    bool EnableAddForce();
    bool EnableBodyGrab();
    void EnableCameraControl();

    Game& game;
    Renderer& renderer;
    DebugOptions& options;

    Camera camera;
    WorldSettings settings;
    World* world;

    float dt;
    Vec2 cursorPos;
    std::vector<Collider*> qr;

    RigidBody* targetBody;
    Collider* targetCollider;
    GrabJoint* cursorJoint;
};

typedef Demo* DemoCreateFunction(Game& game);

struct DemoFrame
{
    const char* name;
    DemoCreateFunction* createFunction;
};

#define MAX_DEMOS 100
extern int32 demo_count;
extern DemoFrame demos[MAX_DEMOS];

} // namespace muli