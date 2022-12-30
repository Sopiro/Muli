#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static bool constrain_linear_motion = true;

class Springs : public Demo
{
public:
    Springs(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;

        RigidBody* g = world->CreateBox(0.3f, 6, RigidBody::Type::static_body);
        g->SetPosition(0.0f, 3.6f);

        float size = 0.3f;
        float bind = 3.0f;
        float length = 2.0f;
        float yStart = 3.6f;
        float yGap = 2.0f;

        RigidBody* b = world->CreateBox(size);
        b->SetPosition(length, yStart + yGap);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), length, 2.0f, 0.05f, b->GetMass());
        if (constrain_linear_motion)
        {
            world->CreatePrismaticJoint(g, b, b->GetPosition(), Vec2{ 1.0f, 0.0f }, -1.0f);
        }
        b->Translate(bind - length, 0.0f);

        b = world->CreateBox(size);
        b->SetPosition(length, yStart);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), length, 2.0f, 0.2f, b->GetMass());
        if (constrain_linear_motion)
        {
            world->CreatePrismaticJoint(g, b, b->GetPosition(), Vec2{ 1.0f, 0.0f }, -1.0f);
        }
        b->Translate(bind - length, 0.0f);

        b = world->CreateBox(size);
        b->SetPosition(length, yStart - yGap);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), length, 2.0f, 0.7f, b->GetMass());
        if (constrain_linear_motion)
        {
            world->CreatePrismaticJoint(g, b, b->GetPosition(), Vec2{ 1.0f, 0.0f }, -1.0f);
        }
        b->Translate(bind - length, 0.0f);

        b = world->CreateBox(size);
        b->SetPosition(-length, yStart + yGap);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), length, 1.0f, 0.2f, b->GetMass());
        if (constrain_linear_motion)
        {
            world->CreatePrismaticJoint(g, b, b->GetPosition(), Vec2{ 1.0f, 0.0f }, -1.0f);
        }
        b->Translate(-(bind - length), 0.0f);

        // Reduce the amplitude by half every second
        float halfLife = 1.0f;
        float frequency = -Log(0.5f) / (halfLife * pi * 2.0f);

        b = world->CreateBox(size);
        b->SetPosition(-length, yStart);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), length, frequency, 1.0f, b->GetMass());
        if (constrain_linear_motion)
        {
            world->CreatePrismaticJoint(g, b, b->GetPosition(), Vec2{ 1.0f, 0.0f }, -1.0f);
        }
        b->Translate(-(bind - length), 0.0f);

        b = world->CreateBox(size);
        b->SetPosition(-length, yStart - yGap);
        world->CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), length, 4.0f, 0.01f, b->GetMass());
        if (constrain_linear_motion)
        {
            world->CreatePrismaticJoint(g, b, b->GetPosition(), Vec2{ 1.0f, 0.0f }, -1.0f);
        }
        b->Translate(-(bind - length), 0.0f);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Springs", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::Checkbox("Constrain linear motion", &constrain_linear_motion))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new Springs(game);
    }
};

DemoFrame springs{ "Springs", Springs::Create };

} // namespace muli
