#if defined(_WIN32) && defined(_DEBUG)
#include <crtdbg.h>
#endif

#include <chrono>

#include "game.h"
#include "window.h"

using namespace muli;

Window* window;
Game* game;
float frameTime;
float deltaTime;
Vec3 clearColor{ 190.0f / 255.0f, 220.0f / 255.0f, 230.0f / 255.0f };

void SetTickRate(int32 tickRate)
{
    tickRate = std::clamp<int32>(tickRate, 30, 300);
    frameTime = 1.0f / tickRate;
}

void Init()
{
#ifdef __EMSCRIPTEN__
    window = Window::Init(1280, 720, "Muli Engine Demo");
    SetTickRate(60);
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
}

void Terminate()
{
    delete game;
}

void Update()
{
    window->BeginFrame(clearColor);
    game->Update(frameTime);
    game->Render();
    window->EndFrame();
}

int main(int argc, char** argv)
{
#if defined(_WIN32) && defined(_DEBUG)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    Init();

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop(Update, 0, 1);
#else
    auto lastTime = std::chrono::steady_clock::now();
    SetTickRate(window->GetRefreshRate());

    while (!window->ShouldClose())
    {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration = currentTime - lastTime;
        float elapsed = duration.count();
        lastTime = currentTime;

        deltaTime += elapsed;
        if (deltaTime > frameTime)
        {
            Update();
            deltaTime -= frameTime;
        }
    }
#endif

    Terminate();

    return 0;
}