#pragma once

#include "camera.h"
#include "common.h"
#include "demo.h"
#include "dynamic_renderer.h"
#include "input.h"
#include "rigidbody_renderer.h"

namespace muli
{

class Application;

struct DebugOptions
{
    bool pause = false;
    bool step = false;
    bool drawOutlineOnly = false;
    bool showBVH = false;
    bool showAABB = false;
    bool showContactPoint = false;
    bool showContactNormal = false;
    bool resetCamera = true;
};

class Game final
{
public:
    Game(Application& app);
    ~Game() noexcept;

    Game(const Game&) noexcept = delete;
    Game& operator=(const Game&) noexcept = delete;

    Game(Game&&) noexcept = delete;
    Game& operator=(Game&&) noexcept = delete;

    void Update(float dt);
    void Render();

    RigidBodyRenderer& GetRigidBodyRenderer();
    DebugOptions& GetDebugOptions();

private:
    Application& app;

    std::vector<Vec2> points{};
    std::vector<Vec2> lines{};
    DynamicRenderer dRenderer{};
    RigidBodyRenderer rRenderer{};

    float time = 0.0f;

    uint32 demoIndex;
    Demo* demo = nullptr;
    DebugOptions options;

    void UpdateProjectionMatrix();
    void UpdateUI();
    void UpdateInput();
    void InitDemo(uint32 demo);
};

inline RigidBodyRenderer& Game::GetRigidBodyRenderer()
{
    return rRenderer;
}

inline DebugOptions& Game::GetDebugOptions()
{
    return options;
}

} // namespace muli