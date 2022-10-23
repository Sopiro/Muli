#include "demo.h"

namespace muli
{

class Frictions : public Demo
{
public:
    Frictions(Game& game)
        : Demo(game)
    {
        float groundFriction = 0.5f;

        RigidBody* ground = world->CreateBox(100.0f, 0.4f, RigidBody::Type::Static);
        ground->SetFriction(groundFriction);

        RigidBody* b = world->CreateCapsule(6.0f, 0.05f, true, RigidBody::Type::Static);
        b->SetPosition(-0.6f, 5.0f);
        b->SetRotation(-0.15f);
        b->SetFriction(groundFriction);

        b = world->CreateCapsule(6.0f, 0.05f, true, RigidBody::Type::Static);
        b->SetPosition(0.0f, 3.0f);
        b->SetRotation(0.15f);
        b->SetFriction(groundFriction);

        b = world->CreateCapsule(6.0f, 0.05f, true, RigidBody::Type::Static);
        b->SetPosition(-0.6f, 1.0f);
        b->SetRotation(-0.15f);
        b->SetFriction(groundFriction);

        b = world->CreateBox(0.1f, 1.1f, RigidBody::Type::Static);
        b->SetPosition(3.1f, 4.3f);
        b = world->CreateBox(0.1f, 1.1f, RigidBody::Type::Static);
        b->SetPosition(-3.7f, 2.3f);

        float xStart = -4.5f;
        float yStart = 7.0f;
        float gap = 0.30f;
        float size = 0.30f;

        std::array<float, 5> frictions = { 0.4f, 0.3f, 0.2f, 0.1f, 0.0f };

        for (uint32 i = 0; i < frictions.size(); i++)
        {
            b = world->CreateBox(size, size);
            b->SetPosition(xStart + (size + gap) * i, yStart);
            b->SetFriction(frictions[i]);
            b->SetLinearVelocity(2.3f, 0.0f);
        }
    }

    static Demo* Create(Game& game)
    {
        return new Frictions(game);
    }
};

DemoFrame frictions{ "Frictions", Frictions::Create };

} // namespace muli
