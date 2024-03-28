#if defined(_WIN32) && defined(_DEBUG)
#include <crtdbg.h>
#endif

#include "application.h"

int main(int argc, char** argv)
{
#if defined(_WIN32) && defined(_DEBUG)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    auto app = muli::Application::Create(1600, 900, "Muli Engine Demo");

    app->Run();

    return 0;
}