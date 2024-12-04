#include "demo.h"

namespace muli
{

class SinglePendulum : public Demo
{
public:
    SinglePendulum(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        RigidBody* b = world->CreateBox(0.6f);
        b->SetPosition(-6.0f, 10.0f);

        world->CreateRevoluteJoint(ground, b, Vec2{ 0.0f, 10.0f }, -1.0f);
    }

    static Demo* Create(Game& game)
    {
        return new SinglePendulum(game);
    }
};

static int index = register_demo("Single pendulum", SinglePendulum::Create, 4);

} // namespace muli
