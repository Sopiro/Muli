#if defined(_WIN32)
#include <crtdbg.h>
#endif

#include "application.h"

int main(int argc, char** argv)
{
#if defined(_WIN32)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

    std::cout << "circle: " << sizeof(muli::Circle) << std::endl;
    std::cout << "capsule: " << sizeof(muli::Capsule) << std::endl;
    std::cout << "polygon: " << sizeof(muli::Polygon) << std::endl;

    std::cout << "Angle joint: " << sizeof(muli::AngleJoint) << std::endl;
    std::cout << "Grab joint: " << sizeof(muli::GrabJoint) << std::endl;
    std::cout << "Distance joint: " << sizeof(muli::DistanceJoint) << std::endl;
    std::cout << "Revolute joint: " << sizeof(muli::RevoluteJoint) << std::endl;
    std::cout << "Prismatic joint: " << sizeof(muli::PrismaticJoint) << std::endl;
    std::cout << "Pulley joint: " << sizeof(muli::PulleyJoint) << std::endl;
    std::cout << "Weld joint: " << sizeof(muli::WeldJoint) << std::endl;

    muli::Application* app = muli::Application::Create(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);

    app->Run();

    delete app;

    return 0;
}