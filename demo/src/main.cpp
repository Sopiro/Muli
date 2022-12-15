#if defined(_WIN32) && defined(_DEBUG)
#include <crtdbg.h>
#endif

#include "application.h"

void PrintSizes()
{
    std::cout << "Circle: " << sizeof(muli::Circle) << '\n';
    std::cout << "Capsule: " << sizeof(muli::Capsule) << '\n';
    std::cout << "Polygon: " << sizeof(muli::Polygon) << '\n';
    std::cout << "Collider: " << sizeof(muli::Collider) << '\n';
    std::cout << "RigidBody: " << sizeof(muli::RigidBody) << '\n';
    std::cout << "Contact: " << sizeof(muli::Contact) << '\n';
    std::cout << "Manifold: " << sizeof(muli::ContactManifold) << '\n';
    std::cout << '\n';

    std::cout << "Angle joint: " << sizeof(muli::AngleJoint) << '\n';
    std::cout << "Distance joint: " << sizeof(muli::DistanceJoint) << '\n';
    std::cout << "Grab joint: " << sizeof(muli::GrabJoint) << '\n';
    std::cout << "Line joint: " << sizeof(muli::LineJoint) << '\n';
    std::cout << "Motor joint: " << sizeof(muli::MotorJoint) << '\n';
    std::cout << "Prismatic joint: " << sizeof(muli::PrismaticJoint) << '\n';
    std::cout << "Pulley joint: " << sizeof(muli::PulleyJoint) << '\n';
    std::cout << "Revolute joint: " << sizeof(muli::RevoluteJoint) << '\n';
    std::cout << "Weld joint: " << sizeof(muli::WeldJoint) << '\n' << std::endl;
}

int main(int argc, char** argv)
{
#if defined(_WIN32) && defined(_DEBUG)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    PrintSizes();

    muli::Application* app = muli::Application::Create(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);

    app->Run();

    delete app;

    return 0;
}