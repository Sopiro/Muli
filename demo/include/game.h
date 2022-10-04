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

    Vec2 GetWorldMousePosition() const;
    DebugOptions& GetDebugOptions();
    void RegisterRenderBody(RigidBody* b);
    std::vector<Vec2>& GetPointList();
    std::vector<Vec2>& GetLineList();
    float GetTime() const;

private:
    Application& app;

    std::vector<Vec2> points{};
    std::vector<Vec2> lines{};
    RigidBodyRenderer rRenderer{};
    DynamicRenderer dRenderer{};

    float time = 0.0f;

    uint32 demoIndex;
    Demo* demo = nullptr;
    DebugOptions options;

    void UpdateProjectionMatrix();
    void UpdateUI();
    void UpdateInput();
    void InitDemo(uint32 demo);
};

inline DebugOptions& Game::GetDebugOptions()
{
    return options;
}

inline std::vector<Vec2>& Game::GetPointList()
{
    return points;
}

inline std::vector<Vec2>& Game::GetLineList()
{
    return lines;
}

inline void Game::RegisterRenderBody(RigidBody* b)
{
    b->OnDestroy = [&](RigidBody* me) -> void { rRenderer.Unregister(me); };
    rRenderer.Register(b);
}

inline Vec2 Game::GetWorldMousePosition() const
{
    return rRenderer.Pick(Input::GetMousePosition());
}

inline float Game::GetTime() const
{
    return time;
}

} // namespace muli