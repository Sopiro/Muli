#include "demo.h"

namespace muli
{

class Springs : public Demo
{
public:
    Springs(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;

        RigidBody* g = world->CreateBox(0.3f, 6, RigidBody::Type::static_body);
        g->SetPosition(0.0f, 3.6f);

        RigidBody* b = world->CreateBox(0.3f);
        b->SetPosition(3.0f, 3.6f + 2.0f);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.05f, b->GetMass());

        b = world->CreateBox(0.3f);
        b->SetPosition(3.0f, 3.6f);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.2f, b->GetMass());

        b = world->CreateBox(0.3f);
        b->SetPosition(3.0f, 3.6f - 2.0f);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.7f, b->GetMass());

        b = world->CreateBox(0.3f);
        b->SetPosition(-3.0f, 3.6f + 2.0f);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 0.5f, 0.2f, b->GetMass());

        // Reduce the amplitude by half every second
        float halfLife = 1.0f;
        float frequency = -Log(0.5f) / (halfLife * MULI_PI * 2.0f);

        b = world->CreateBox(0.3f);
        b->SetPosition(-3.0f, 3.6f);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, frequency, 1.0f, b->GetMass());

        b = world->CreateBox(0.3f);
        b->SetPosition(-3.0f, 3.6f - 2.0f);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 2.0f, 0.01f, b->GetMass());
    }

    static Demo* Create(Game& game)
    {
        return new Springs(game);
    }
};

DemoFrame springs{ "Springs", Springs::Create };

} // namespace muli
