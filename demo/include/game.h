#pragma once

#include "camera.h"
#include "common.h"
#include "demo.h"
#include "input.h"
#include "options.h"
#include "renderer.h"

namespace muli
{

class Application;

class Game final : NonCopyable,
                   ColliderDestroyCallback
{
public:
    Game(Application& app);
    ~Game() noexcept;

    void Update(float dt);
    void Render();

    Vec2 GetWorldCursorPosition() const;
    DebugOptions& GetDebugOptions();
    Renderer& GetRenderer();

    float GetTime() const;
    void RestartDemo();
    void NextDemo();
    void PrevDemo();

private:
    Application& app;

    Renderer renderer;

    float time = 0.0f;

    size_t demoCount;
    size_t newIndex;
    size_t demoIndex;
    Demo* demo = nullptr;
    bool restart = false;
    DebugOptions options;

    void UpdateProjectionMatrix();
    void UpdateUI();
    void UpdateInput();
    void InitDemo(int32 demo);
    virtual void OnColliderDestroy(Collider* me) override;
};

inline DebugOptions& Game::GetDebugOptions()
{
    return options;
}

inline Vec2 Game::GetWorldCursorPosition() const
{
    return renderer.Pick(Input::GetMousePosition());
}

inline float Game::GetTime() const
{
    return time;
}

inline void Game::RestartDemo()
{
    restart = true;
    newIndex = demoIndex;
}

inline void Game::NextDemo()
{
    newIndex = (demoIndex + 1) % demoCount;
    restart = true;
}

inline void Game::PrevDemo()
{
    newIndex = (demoIndex - 1 + demoCount) % demoCount;
    restart = true;
}

inline Renderer& Game::GetRenderer()
{
    return renderer;
}

inline void Game::OnColliderDestroy(Collider* me)
{
    muliNotUsed(me);
}

} // namespace muli