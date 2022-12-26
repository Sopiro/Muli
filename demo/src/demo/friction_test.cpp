#include "demo.h"

namespace muli
{

class FrictionTest : public Demo
{
public:
    FrictionTest(Game& game)
        : Demo(game)
    {
        float groundFriction = 0.5f;

        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        ground->SetFriction(groundFriction);

        RigidBody* b = world->CreateCapsule(6.5f, 0.05f, true, RigidBody::Type::static_body);
        b->SetPosition(-0.6f, 5.0f);
        b->SetRotation(-0.15f);
        b->SetFriction(groundFriction);

        b = world->CreateCapsule(6.5f, 0.05f, true, RigidBody::Type::static_body);
        b->SetPosition(0.0f, 3.0f);
        b->SetRotation(0.15f);
        b->SetFriction(groundFriction);

        b = world->CreateCapsule(6.5f, 0.05f, true, RigidBody::Type::static_body);
        b->SetPosition(-0.6f, 1.0f);
        b->SetRotation(-0.15f);
        b->SetFriction(groundFriction);

        b = world->CreateBox(0.1f, 1.1f, RigidBody::Type::static_body);
        b->SetPosition(3.2f, 4.3f);
        b = world->CreateBox(0.1f, 1.1f, RigidBody::Type::static_body);
        b->SetPosition(-3.8f, 2.3f);

        float xStart = -4.5f;
        float yStart = 7.0f;
        float gap = 0.30f;
        float size = 0.30f;

        std::array<float, 5> frictions = { 0.4f, 0.2f, 0.12f, 0.04f, 0.0f };

        for (size_t i = 0; i < frictions.size(); ++i)
        {
            b = world->CreateBox(size, size);
            b->SetPosition(xStart + (size + gap) * i, yStart);
            b->SetFriction(frictions[i]);
            b->SetLinearVelocity(2.0f, 0.0f);
        }
    }

    static Demo* Create(Game& game)
    {
        return new FrictionTest(game);
    }
};

DemoFrame friction_test{ "Friction test", FrictionTest::Create };

} // namespace muli
