#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ComputeDistanceShape : public Demo
{
public:
    ComputeDistanceShape(Game& game)
        : Demo(game)
    {
        options.draw_outline_only = true;
        options.show_contact_normal = true;
        options.show_contact_point = true;
        settings.apply_gravity = false;

        float size = 1.0f;
        float range = size * 0.7f;

        float r = LinearRand(0.0f, 3.0f);
        RigidBody* b;

        if (r < 1.0f)
        {
            b = world->CreateBox(size);
        }
        else if (r < 2.0f)
        {
            b = world->CreateCircle(size / 2.0f);
        }
        else
        {
            b = world->CreateCapsule(size, size / 2.0f);
        }

        b->SetPosition(-1.0f, 0.0f);
        b->SetRotation(LinearRand(0.0f, MULI_PI));

        r = LinearRand(0.0f, 3.0f);

        if (r < 1.0f)
        {
            b = world->CreateBox(size);
        }
        else if (r < 2.0f)
        {
            b = world->CreateCircle(size / 2.0f);
        }
        else
        {
            b = world->CreateCapsule(size, size / 2.0f);
        }

        b->SetPosition(1.0f, 0.0f);
        b->SetRotation(LinearRand(0.0f, MULI_PI));

        camera.position = 0.0f;
        camera.scale = 0.5f;
    }

    void UpdateUI() override
    {
        if (world->GetBodyCount() > 1)
        {
            RigidBody* b1 = world->GetBodyList();
            RigidBody* b2 = b1->GetNext();

            const Shape* s1 = b1->GetColliderList()->GetShape();
            const Shape* s2 = b2->GetColliderList()->GetShape();

            const Transform& tf1 = b1->GetTransform();
            const Transform& tf2 = b2->GetTransform();

            ClosestFeatures f;

            float distance = ComputeDistance(s1, tf1, s2, tf2, &f);

            if (distance > 0.0f)
            {
                std::vector<Vec2>& pl = game.GetPointList();
                pl.push_back(f.pointA);
                pl.push_back(f.pointB);

                ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
                ImGui::Begin("Overlay", NULL,
                             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
                ImGui::TextColored(ImColor{ 12, 11, 14 }, "Distance: %f", distance);
                ImGui::End();
            }
            else
            {
                ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
                ImGui::Begin("Overlay", NULL,
                             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
                ImGui::TextColored(ImColor{ 12, 11, 14 }, "%s", "Collide!");
                ImGui::End();
            }
        }
    }

    ~ComputeDistanceShape()
    {
        options.draw_outline_only = false;
        options.show_contact_normal = false;
        options.show_contact_point = false;
    }

    static Demo* Create(Game& game)
    {
        return new ComputeDistanceShape(game);
    }
};

DemoFrame distance_test{ "Distance between shapes", ComputeDistanceShape::Create };

} // namespace muli