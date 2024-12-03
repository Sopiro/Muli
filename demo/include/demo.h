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

    virtual void OnJointDestroy(Joint* me) override;

    World& GetWorld();
    WorldSettings& GetWorldSettings();
    Camera& GetCamera();
    RigidBody* GetTargetBody();
    Collider* GetTargetCollider();

protected:
    void FindTargetBody();
    void EnableKeyboardShortcut();
    void EnableBodyCreate();
    bool EnablePolygonCreateConvexHull();
    bool EnablePolygonCreateDecomposition();
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
    Vec2 screenBounds;

    RigidBody* targetBody;
    Collider* targetCollider;
    GrabJoint* cursorJoint;
};

inline void Demo::OnJointDestroy(Joint* me)
{
    cursorJoint = nullptr;
}

inline World& Demo::GetWorld()
{
    return *world;
}

inline WorldSettings& Demo::GetWorldSettings()
{
    return settings;
}

inline Camera& Demo::GetCamera()
{
    return camera;
}

inline RigidBody* Demo::GetTargetBody()
{
    return targetBody;
}

inline Collider* Demo::GetTargetCollider()
{
    return targetCollider;
}

typedef Demo* DemoCreateFunction(Game& game);

struct DemoFrame
{
    const char* name;
    DemoCreateFunction* createFunction;
    int index;
};

inline std::vector<DemoFrame> demos;

inline int32 register_demo(const char* name, DemoCreateFunction* createFunction, int32 index = 0)
{
    demos.push_back(DemoFrame{ name, createFunction, index });
    return (int32)demos.size();
}

inline void sort_demos(std::vector<DemoFrame>& demos)
{
    std::sort(demos.begin(), demos.end(), [&demos](DemoFrame& l, DemoFrame& r) { return l.index < r.index; });
}

} // namespace muli