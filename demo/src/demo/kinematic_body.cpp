#include "demo.h"

namespace muli
{

static float speed = pi / 2;

class KinematicBody : public Demo
{
    RigidBody* k;

public:
    KinematicBody(Game& game)
        : Demo(game)
    {
        float size = 15.0f;
        float halfSize = size / 2.0f;
        float wallWidth = 0.4f;
        float wallRadius = wallWidth / 2.0f;

        world->CreateCapsule(
            Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, identity, RigidBody::static_body
        );
        world->CreateCapsule(
            Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, identity, RigidBody::static_body
        );
        world->CreateCapsule(
            Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, identity, RigidBody::static_body
        );

        float r = 0.22f;

        for (int32 i = 0; i < 500; ++i)
        {
            RigidBody* b = world->CreateCircle(r);
            b->SetPosition(
                Rand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f, Rand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f
            );
            b->SetRotation(Rand(0.0f, pi * 2.0f));
        }

        k = world->CreateCapsule(size * 0.9f, 0.15f, true, identity, RigidBody::kinematic_body);
        k->CreateCapsuleCollider(size * 0.9f, 0.15f, false);
        k->SetAngularVelocity(speed);

        camera.position = { 0.0f, 0.0f };
        camera.scale = { 3.f, 3.f };
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Kinematic", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::SliderFloat("Speed", &speed, 0, 10))
            {
                k->Awake();
                k->SetAngularVelocity(speed);
            }
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new KinematicBody(game);
    }
};

static int index = register_demo("Kinematic body", KinematicBody::Create, 20);

} // namespace muli
