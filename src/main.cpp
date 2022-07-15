#pragma warning(disable:4819)

#if defined(_WIN32)
#include <crtdbg.h>
#endif

#include "spe/application.h"

int main(int argc, char** argv)
{
#if defined(_WIN32)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

    std::srand(std::time(0));
    spe::Application* app = spe::Application::Create(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);

    app->Run();

    delete app;

    return 0;
}