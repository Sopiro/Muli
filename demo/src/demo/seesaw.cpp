#include "demo.h"

namespace muli
{

class Seesaw : public Demo
{
public:
    Seesaw(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        RigidBody* seesaw = world->CreateCapsule(6.0f, 0.05f, true);
        seesaw->SetPosition(0.0f, 0.45f);
        seesaw->GetColliderList()->SetMass(10.0f);

        world->CreateRevoluteJoint(ground, seesaw, seesaw->GetPosition(), -1);

        RigidBody* b = world->CreateCircle(0.2f);
        b->SetPosition(-2.5f, 1.0f);

        b = world->CreateBox(0.2f);
        b->SetPosition(-2.8f, 1.0f);
        b->GetColliderList()->SetMass(1.0f);

        b = world->CreateBox(0.5f);
        b->SetPosition(2.5f, 5.0f);
        b->GetColliderList()->SetMass(30.0f);

        b = world->CreateCapsule(1.0f, 0.5f, false, RigidBody::Type::dynamic_body);
        b->SetPosition(-2.5, 240.0f);
        b->SetContinuous(true);
    }

    static Demo* Create(Game& game)
    {
        return new Seesaw(game);
    }
};

DemoFrame seesaw{ "Seesaw", Seesaw::Create };

} // namespace muli
