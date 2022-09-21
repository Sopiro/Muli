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

    Camera& GetCamera();

private:
    Application& app;

    Camera camera{};

    std::vector<Vec2> points{};
    std::vector<Vec2> lines{};
    DynamicRenderer dRenderer{};
    RigidBodyRenderer rRenderer{};

    float time = 0.0f;
    float simulationDeltaTime = 0.0f;
    GrabJoint* gj = nullptr;
    Vec2 mpos{ 0.0f };

    bool pause = false;
    bool step = false;
    bool drawOutlineOnly = false;
    bool showBVH = false;
    bool showAABB = false;
    bool showContactPoint = false;
    bool showContactNormal = false;
    bool resetCamera = true;

    std::vector<RigidBody*> qr;
    uint32 demoIndex;
    Demo* demo = nullptr;

    void UpdateProjectionMatrix();
    void UpdateUI();
    void HandleInput();
    void InitDemo(uint32 demo);
    void Reset();
};

inline Camera& Game::GetCamera()
{
    return camera;
}

} // namespace muli