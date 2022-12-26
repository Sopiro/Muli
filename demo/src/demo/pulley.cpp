#include "demo.h"

namespace muli
{

class Pulley : public Demo
{
public:
    Pulley(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        RigidBody* b1 = world->CreateBox(0.5f);
        b1->SetPosition(-1, 3);
        RigidBody* b2 = world->CreateBox(0.5f);
        b2->SetPosition(1, 3);

        Vec2 ga = Vec2{ -1.0f, 5.0f };
        Vec2 gb = Vec2{ 1.0f, 5.0f };
        Vec2 offset = Vec2{ 0.0f, 0.25f };
        float ratio = 1.0f;

        world->CreatePulleyJoint(b1, b2, b1->GetPosition() + offset, b2->GetPosition() + offset, ga, gb, ratio);

        world->CreateCapsule(ga, gb, 0.075f, RigidBody::Type::static_body);
    }

    static Demo* Create(Game& game)
    {
        return new Pulley(game);
    }
};

DemoFrame pulley{ "Pulley", Pulley::Create };

} // namespace muli
