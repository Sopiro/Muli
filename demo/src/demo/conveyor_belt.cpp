#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static float speed = 2.0f;

class ConveyorBelt : public Demo
{
public:
    ConveyorBelt(Game& game)
        : Demo(game)
    {
        RigidBody* b = world->CreateBox(0.2f, 5.0f, RigidBody::Type::static_body);
        b->SetPosition(6.2f, 3.5f);

        b = world->CreateBox(3.0f, 0.2f, RigidBody::Type::static_body);
        b->SetPosition(-4.0f, 5.0f);
        b->SetSurfaceSpeed(speed);

        b = world->CreateBox(5.5f, 0.2f, RigidBody::Type::static_body);
        b->SetPosition(0.5f, 4.0f);
        b->SetRotation(0.2f);
        b->SetSurfaceSpeed(speed);

        RigidBody* br = world->CreateBox(3.5f, 0.2f, RigidBody::Type::static_body);
        br->SetPosition(4.55f, 1.0f);
        br->SetSurfaceSpeed(-speed);

        RigidBody* bl = world->CreateBox(2.0f, 0.2f, RigidBody::Type::static_body);
        bl->SetPosition(-4.55f, 1.0f);
        bl->SetSurfaceSpeed(-speed);

        float xStart = -3.3f;
        float yStart = 1.05f;
        float radius = 0.05f;
        float length = 0.4f - radius * 2.0f;
        float gap = 0.05f;

        RigidBody* b0 = world->CreateCapsule(length, radius, true);
        b0->SetPosition(xStart, yStart);
        b0->SetSurfaceSpeed(speed);

        Joint* j = world->CreateRevoluteJoint(b0, bl, b0->GetPosition() - Vec2{ 0.2f, 0.0f }, 12.0f, 1.0f, 1.0f);

        for (int32 i = 1; i < 14; ++i)
        {
            RigidBody* b1 = world->CreateCapsule(length, radius, true);
            b1->SetSurfaceSpeed(speed);
            b1->SetPosition(xStart + (length + radius * 2.0f + gap) * i, yStart);

            j = world->CreateRevoluteJoint(b0, b1, (b0->GetPosition() + b1->GetPosition()) * 0.5f, 12.0f, 1.0f, 1.0f);

            b0 = b1;
        }

        j = world->CreateRevoluteJoint(br, b0, b0->GetPosition() + Vec2{ 0.2f, 0.0f }, 12.0f, 1.0f, 1.0f);
    }

    float lastTime = 0.0f;

    void Step() override
    {
        float currentTime = game.GetTime();
        if (!options.pause && currentTime - lastTime > 0.5f)
        {
            RigidBody* c = world->CreateRandomConvexPolygon(0.3f);
            c->SetPosition(LinearRand(-5.5f, -3.0f), 7.0f);

            game.RegisterRenderBody(c);

            lastTime = currentTime;
        }

        Demo::Step();
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Conbeyor belt", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::SliderFloat("Surface speed", &speed, 0.0f, 10.0f, "%.2f m/s");
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new ConveyorBelt(game);
    }
};

DemoFrame conveyor_belt{ "Conbeyor belt", ConveyorBelt::Create };

} // namespace muli
