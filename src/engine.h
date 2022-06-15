#include "common.h"
#include "window.h"

class Engine
{
public:
    Engine() = delete;

    Engine(int width, int height, std::string title);

    ~Engine();

    void run();

private:
    Window window;
};