#include "demo.h"
#include "window.h"

namespace muli
{

static int32 count = 10;
static float speed = 1;

static bool multiple = false;

static const int32 num_shapes = 3;
static const char* shapes[num_shapes] = { "Box", "Circle", "Capsule" };
static int32 shape_index = 0;

class Press : public Demo
{

    RigidBody *p1, *p2;

public:
    Press(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;

        float l = 15.0f;
        float r = 0.3f;
        float offset = 1.0f;

        float size = 0.6f;
        float gap = 0.05f;
        float start = r + size / 2.0f + gap;

        float spacing = 2;

        using Func = RigidBody*(World * world, float size);

        static Func* creator1 = [](World* world, float size) { return world->CreateBox(size); };
        static Func* creator2 = [](World* world, float size) { return world->CreateCircle(size / 2); };
        static Func* creator3 = [](World* world, float size) { return world->CreateCapsule(size / 2, size / 4); };

        Func* creators[3] = { creator1, creator2, creator3 };

        for (int32 i = 0; i < count; ++i)
        {
            RigidBody* b = creators[shape_index](world, size);
            b->SetPosition(0, start + i * (size + gap));
        }

        if (multiple)
        {
            for (int32 i = 0; i < count; ++i)
            {
                RigidBody* b = creators[(shape_index + 1) % num_shapes](world, size);
                b->SetPosition(-spacing, start + i * (size + gap));
            }

            for (int32 i = 0; i < count; ++i)
            {
                RigidBody* b = creators[(shape_index + 2) % num_shapes](world, size);
                b->SetPosition(spacing, start + i * (size + gap));
            }
        }

        float h = count * (size + gap) - gap;
        float h2 = Max(10, count) * (size + gap) - gap;

        camera.position.Set(0.0f, h / 2.0f);
        camera.scale.Set(h2 / screenBounds.y * 2);

        p1 = world->CreateCapsule(l, r, true, identity, RigidBody::kinematic_body);
        p1->SetPosition({ 0, -offset });
        p2 = world->CreateCapsule(l, r, true, identity, RigidBody::kinematic_body);
        p2->SetPosition({ 0, h + size + 2 * gap + offset });

        p1->SetLinearVelocity({ 0, speed });
        p2->SetLinearVelocity({ 0, -speed });
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Press", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::SliderInt("Count", &count, 1, 100);
            if (ImGui::SliderFloat("Speed", &speed, 0, 10.0f))
            {
                p1->Awake();
                p2->Awake();

                p1->SetLinearVelocity({ 0, speed });
                p2->SetLinearVelocity({ 0, -speed });
            }
            ImGui::Separator();
            if (ImGui::Checkbox("Multiple lines", &multiple))
            {
                game.RestartDemo();
            }

            ImGui::Text("Shape");
            if (ImGui::Combo("##Shape", &shape_index, shapes, num_shapes))
            {
                game.RestartDemo();
            }
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new Press(game);
    }
};

static int index = register_demo("Press", Press::Create, 55);

} // namespace muli
