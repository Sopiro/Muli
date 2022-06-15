#include "common.h"

class Window
{
public:
    Window(int width, int height, std::string title);

    bool errorOccurred();

    bool shouldClose();

    void pollEvents();

    ~Window();
private:
    bool error;
    GLFWwindow* window;
};