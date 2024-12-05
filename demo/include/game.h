#pragma once

#include "camera.h"
#include "common.h"
#include "input.h"
#include "options.h"
#include "renderer.h"

namespace muli
{

class Demo;

class Game final : NonCopyable,
                   ColliderDestroyCallback
{
public:
    Game();
    ~Game() noexcept;

    void Update(float dt);
    void FixedUpdate();
    void Render();

    Vec2 GetWorldCursorPosition() const;
    DebugOptions& GetDebugOptions();
    Renderer& GetRenderer();

    float GetTime() const;
    void RestartDemo();
    void NextDemo();
    void PrevDemo();

    float GetWindowScale() const;
    void SetWindowScale(float newScale);

    float GetFixedDeltaTime() const;
    void SetFixedDeltaTime(float fixedDeltaTime);

private:
    Renderer renderer;

    float scale;
    float fixedDeltaTime;
    float time;
    float dt;

    size_t demoCount;
    size_t newIndex;
    size_t demoIndex;

    Demo* demo = nullptr;
    bool restart = false;

    DebugOptions options;

    void UpdateProjectionMatrix();
    void UpdateUI();
    void UpdateInput();
    void InitDemo(size_t demo);
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

inline float Game::GetWindowScale() const
{
    return scale;
}

inline void Game::SetWindowScale(float newScale)
{
    scale = newScale;
}

inline float Game::GetFixedDeltaTime() const
{
    return fixedDeltaTime;
}

inline void Game::SetFixedDeltaTime(float newFixedDeltaTime)
{
    fixedDeltaTime = newFixedDeltaTime;
}

inline Renderer& Game::GetRenderer()
{
    return renderer;
}

inline void Game::OnColliderDestroy(Collider* me)
{
    MuliNotUsed(me);
}

} // namespace muli