#include "demo.h"
#include "game.h"

namespace muli
{

class ConveyorBelt : public Demo
{
public:
    ConveyorBelt(Game& game)
        : Demo(game)
    {
        RigidBody* b = world->CreateBox(3.0f, 0.2f, RigidBody::Type::static_body);
        b->SetPosition(-4.0f, 5.0f);
        b->SetSurfaceSpeed(2.0f);

        b = world->CreateBox(5.5f, 0.2f, RigidBody::Type::static_body);
        b->SetPosition(0.5f, 4.0f);
        b->SetRotation(0.2f);
        b->SetSurfaceSpeed(2.0f);

        RigidBody* br = world->CreateBox(3.5f, 0.2f, RigidBody::Type::static_body);
        br->SetPosition(4.55f, 1.0f);
        br->SetSurfaceSpeed(-2.0f);

        RigidBody* bl = world->CreateBox(2.0f, 0.2f, RigidBody::Type::static_body);
        bl->SetPosition(-4.55f, 1.0f);
        bl->SetSurfaceSpeed(-2.0f);

        float xStart = -3.3f;
        float yStart = 1.05f;
        float radius = 0.05f;
        float length = 0.4f - radius * 2.0f;
        float gap = 0.05f;

        RigidBody* b0 = world->CreateCapsule(length, radius, true);
        b0->SetPosition(xStart, yStart);
        b0->SetSurfaceSpeed(3.0f);

        Joint* j = world->CreateRevoluteJoint(b0, bl, b0->GetPosition() - Vec2{ 0.2f, 0.0f });

        for (uint32 i = 1; i < 14; ++i)
        {
            RigidBody* b1 = world->CreateCapsule(length, radius, true);
            b1->SetSurfaceSpeed(3.0f);
            b1->SetPosition(xStart + (length + radius * 2.0f + gap) * i, yStart);

            j = world->CreateRevoluteJoint(b0, b1, (b0->GetPosition() + b1->GetPosition()) * 0.5f);

            b0 = b1;
        }

        j = world->CreateRevoluteJoint(br, b0, b0->GetPosition() + Vec2{ 0.2f, 0.0f });
    }

    void Step() override
    {
        static float lastTime = game.GetTime();

        float currentTime = game.GetTime();
        if (currentTime - lastTime > 0.5f)
        {
            RigidBody* c = world->CreateRandomConvexPolygon(0.3f);
            c->SetPosition(LinearRand(-5.5f, -3.0f), 7.0f);
            c->SetRestitution(0.1f);

            game.RegisterRenderBody(c);

            lastTime = currentTime;
        }

        Demo::Step();
    }

    static Demo* Create(Game& game)
    {
        return new ConveyorBelt(game);
    }
};

DemoFrame conveyor_belt{ "Conbeyor belt", ConveyorBelt::Create };

} // namespace muli
