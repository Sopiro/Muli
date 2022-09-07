#pragma once

#include "camera.h"
#include "common.h"
#include "dynamic_renderer.h"
#include "input.h"
#include "rigidbody_renderer.h"

namespace spe
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
    void HandleInput();
    void Render();

    Camera& GetCamera();

private:
    Application& app;

    Settings settings{};
    World* world;

    Camera camera{};
    RigidBodyRenderer rRenderer{};

    std::vector<glm::vec2> points{};
    std::vector<glm::vec2> lines{};
    DynamicRenderer dRenderer{};

    float time = 0.0f;
    float simulationDeltaTime = 0.0f;
    GrabJoint* gj = nullptr;
    glm::vec2 mpos{ 0.0f };

    bool pause = false;
    bool step = false;
    bool drawOutlineOnly = false;
    bool showBVH = false;
    bool showContactPoint = false;
    bool showContactNormal = false;
    bool resetCamera = true;

    std::vector<std::pair<std::string, std::function<void(Game&, World&, Settings&)>>> demos;
    uint32_t currentDemo;
    std::string demoTitle;

    void UpdateProjectionMatrix();
    void InitSimulation(uint32_t demo);
    void Reset();
};

inline Camera& Game::GetCamera()
{
    return camera;
}

} // namespace spe