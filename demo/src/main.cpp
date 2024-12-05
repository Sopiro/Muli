#if defined(_WIN32) && defined(_DEBUG)
#include <crtdbg.h>
#endif

#include <chrono>

#include "game.h"
#include "window.h"

using namespace muli;

static Window* window;
static Game* game;

static int32 frameRate;
static int32 updateRate;
static float targetFrameTime;
static float targetUpdateTime;

static Vec3 clearColor = { 190.0f / 255.0f, 220.0f / 255.0f, 230.0f / 255.0f };

int32 GetFrameRate()
{
    return frameRate;
}

void SetFrameRate(int32 newFrameRate)
{
    frameRate = Clamp(newFrameRate, 30, 300);
    targetFrameTime = 1.0f / frameRate;
}

int32 GetUpdateRate()
{
    return updateRate;
}

void SetUpdateRate(int32 newUpdateRate)
{
    updateRate = Clamp(newUpdateRate, 30, 300);
    targetUpdateTime = 1.0f / updateRate;

    game->SetFixedDeltaTime(targetUpdateTime);
}

void Init()
{
#ifdef __EMSCRIPTEN__
    window = Window::Init(1280, 720, "Muli Engine Demo");
#else
    window = Window::Init(1600, 900, "Muli Engine Demo");
#endif

    // Enable culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    // Enable blend
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    game = new Game();

    SetFrameRate(window->GetRefreshRate());
    SetUpdateRate(window->GetRefreshRate());
}

void Terminate()
{
    delete game;
}

void MainLoop()
{
    static float frameTime = 0;
    static float updateTime = 0;
    static auto lastTime = std::chrono::steady_clock::now();

    auto currentTime = std::chrono::steady_clock::now();

    std::chrono::duration<float> duration = currentTime - lastTime;
    float elapsed = duration.count();
    lastTime = currentTime;

    frameTime += elapsed;
    updateTime += elapsed;

    if (updateTime > targetUpdateTime)
    {
        game->FixedUpdate();

        updateTime -= targetUpdateTime;
    }

    if (frameTime > targetFrameTime)
    {
        window->BeginFrame(clearColor);
        {
            game->Update(frameTime);
            game->Render();
        }
        window->EndFrame();

        frameTime -= targetFrameTime;
    }
}

int main(int argc, char** argv)
{
#if defined(_WIN32) && defined(_DEBUG)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    Init();

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop(MainLoop, 0, 1);
#else
    while (!window->ShouldClose())
    {
        MainLoop();
    }
#endif

    Terminate();

    return 0;
}