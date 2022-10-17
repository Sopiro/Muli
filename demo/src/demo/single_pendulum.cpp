#include "demo.h"

namespace muli
{

class SinglePendulum : public Demo
{
public:
    SinglePendulum(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

        RigidBody* b = world->CreateBox(0.3f);
        b->SetPosition(-3.0f, 5.0f);

        world->CreateRevoluteJoint(ground, b, Vec2{ 0.0f, 5.0f }, -1.0f);
    }

    static Demo* Create(Game& game)
    {
        return new SinglePendulum(game);
    }
};

DemoFrame single_pendulum{ "Single pendulum", SinglePendulum::Create };

} // namespace muli
