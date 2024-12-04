#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ContinuousTest2 : public Demo
{
    static inline bool drawTrajectory = true;
    static inline int32 selection = 1;
    static inline const char* items[] = { "Circle", "Box", "Capsule", "Rounded polygon", "Random" };

public:
    RigidBody* target;

    ContinuousTest2(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        float start = 0.5f;
        float size = 0.3f;
        float gap = 0.25f;

        switch (selection)
        {
        case 0:
            target = world->CreateCircle(0.2f);
            break;
        case 1:
            target = world->CreateBox(0.4f);
            break;
        case 2:
            target = world->CreateCapsule(1.2f, 0.05f);
            break;
        case 3:
            target = world->CreateRandomConvexPolygon(0.28f, 6, RigidBody::Type::dynamic_body, 0.05f);
            target->UserData = (void*)((size_t)target->UserData | UserFlag::render_polygon_radius);
            break;
        case 4:
            target = world->CreateRandomConvexPolygon(0.28f, (int32)Rand(6, 8));
            break;

        default:
            MuliAssert(false);
            break;
        }

        target->SetRotation(Rand(-pi / 2.0f, pi / 2.0f));
        target->SetPosition(-1.0f, 3.0f);
        target->SetContinuous(true);
        target->SetLinearVelocity(100.0f, 0.0f);

        float h = screenBounds.y / 2.0f;
        world->CreateCapsule(Vec2{ h, 6.0f }, Vec2{ h, 0.0f }, 0.05f, RigidBody::Type::static_body);
        world->CreateCapsule(Vec2{ h, 6.0f }, Vec2{ -1.0f, 6.0f }, 0.05f, RigidBody::Type::static_body);

        settings.continuous = true;
    }

    void Render() override
    {
        if (drawTrajectory)
        {
            Transform t = target->GetTransform();
            t.rotation = t.rotation.GetAngle() + target->GetAngularVelocity() * settings.step.dt;
            t.position += target->GetLinearVelocity() * settings.step.dt;

            Renderer::DrawMode drawMode;
            drawMode.fill = false;
            drawMode.outline = true;

            if (target->GetWorld())
            {
                if (((uint64)target->UserData) & UserFlag::render_polygon_radius)
                {
                    drawMode.rounded = true;
                    game.GetRenderer().DrawShape(target->GetColliderList()->GetShape(), t, drawMode);
                }
                else
                {
                    game.GetRenderer().DrawShape(target->GetColliderList()->GetShape(), t, drawMode);
                }
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Continuous test 2", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Shapes");
            ImGui::PushID(0);
            if (ImGui::ListBox("", &selection, items, IM_ARRAYSIZE(items)))
            {
                game.RestartDemo();
            }
            ImGui::PopID();

            ImGui::Checkbox("Draw trajectory", &drawTrajectory);
        }
        ImGui::End();
    }

    ~ContinuousTest2() {}

    static Demo* Create(Game& game)
    {
        return new ContinuousTest2(game);
    }
};

static int index = register_demo("Continuous test 2", ContinuousTest2::Create, 49);

} // namespace muli