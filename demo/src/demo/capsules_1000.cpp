#include "demo.h"

namespace muli
{
static bool rotate = false;
static float speed = 0.5f;

class Capsules1000 : public Demo
{
    RigidBody* frame;

public:
    Capsules1000(Game& game)
        : Demo(game)
    {
        float size = 15.0f;
        float halfSize = size / 2.0f;
        float wallWidth = 0.4f;
        float wallRadius = wallWidth / 2.0f;

        frame = world->CreateEmptyBody(identity, RigidBody::kinematic_body);

        frame->CreateCapsuleCollider(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius);
        frame->CreateCapsuleCollider(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius);
        frame->CreateCapsuleCollider(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius);
        frame->CreateCapsuleCollider(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius);

        float r = 0.3f;

        for (int32 i = 0; i < 1000; ++i)
        {
            RigidBody* c = world->CreateCapsule(r, r / 2.0f);
            c->SetPosition(
                Rand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f, Rand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f
            );
            c->SetRotation(Rand(0.0f, pi * 2.0f));
        }

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };

        frame->SetAngularVelocity(speed * rotate);
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Capsules 1000", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::Checkbox("Rotate frame", &rotate))
            {
                frame->SetAngularVelocity(speed * rotate);
            }

            if (ImGui::SliderFloat("Speed", &speed, 0.0f, 3.14f, "%.2f rad/s"))
            {
                frame->SetAngularVelocity(speed * rotate);
            }
        }
        ImGui::End();
    }
    static Demo* Create(Game& game)
    {
        return new Capsules1000(game);
    }
};

static int index = register_demo("1000 Capsules", Capsules1000::Create, 16);

} // namespace muli
