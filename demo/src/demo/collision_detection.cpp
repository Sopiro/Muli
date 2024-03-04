#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

static const char* items[] = { "Circle", "Capsule", "Polygon" };

class CollisionDetection : public Demo
{
public:
    Transform tf1 = identity, tf2 = identity;
    std::unique_ptr<Shape> shape1, shape2;

    int item1 = 1;
    int item2 = 2;

    bool collide = false;
    ContactManifold manifold;

    CollisionDetection(Game& game)
        : Demo(game)
    {
        UpdateShape1();
        UpdateShape2();

        camera.position.SetZero();

        tf1.position.Set(-0.4f, 0.0f);
        tf2.position.Set(0.4f, 0.0f);
        // tf2.rotation = 0.5f;
    }

    void UpdateUI() override
    {
        // ImGui::ShowDemoWindow();

        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
        if (ImGui::Begin("Collision detection", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar))
        {
            ImGui::SetNextItemWidth(100);
            if (ImGui::Combo("shape1", &item1, items, IM_ARRAYSIZE(items)))
            {
                UpdateShape1();
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            if (ImGui::Combo("shape2", &item2, items, IM_ARRAYSIZE(items)))
            {
                UpdateShape2();
            }

            ImGui::SetNextItemWidth(100);
            ImGui::DragFloat2("pos1", &tf1.position.x, 0.01f);

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            ImGui::DragFloat2("pos2", &tf2.position.x, 0.01f);

            static float r1, r2;
            ImGui::SetNextItemWidth(100);
            if (ImGui::DragFloat("rot1", &r1, 0.01f))
            {
                tf1.rotation = r1;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            if (ImGui::DragFloat("rot2", &r2, 0.01f))
            {
                tf2.rotation = r2;
            }
        }
        ImGui::End();

        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, Window::Get().GetWindowSize().y - 5 }, ImGuiCond_Always,
                                { 1.0f, 1.0f });
        ImGui::Begin("notice", NULL,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Contact normal is pointing from refernce body to incident body");
        ImGui::End();
    }

    void UpdateInput() override
    {
        cursorPos = game.GetWorldCursorPosition();

        static bool grab = false;
        static int grabbed;
        static Vec2 offset;

        if (!grab && Input::IsMouseDown(0))
        {
            if (shape1->TestPoint(tf1, cursorPos))
            {
                grabbed = 1;
                offset = cursorPos - tf1.position;
                grab = true;
            }
            else if (shape2->TestPoint(tf2, cursorPos))
            {
                grabbed = 2;
                offset = cursorPos - tf2.position;
                grab = true;
            }
        }

        if (grab)
        {
            if (Input::IsMouseReleased(0))
            {
                grab = false;
                return;
            }

            if (grabbed == 1)
            {
                tf1.position = cursorPos - offset;
            }
            else
            {
                tf2.position = cursorPos - offset;
            }
        }
    }

    void Step() override
    {
        collide = Collide(shape1.get(), tf1, shape2.get(), tf2, &manifold);
    }

    void Render() override
    {
        int32 color1 = -1;
        int32 color2 = -1;
        if (collide)
        {
            color2 = 0;
            if (manifold.featureFlipped)
            {
                std::swap(color1, color2);
            }
        }

        renderer.DrawShape(shape1.get(), tf1, Renderer::DrawMode{ .colorIndex = color1 });
        renderer.DrawShape(shape2.get(), tf2, Renderer::DrawMode{ .colorIndex = color2 });

        if (collide)
        {
            for (int32 i = 0; i < manifold.contactCount; ++i)
            {
                Vec2 p1 = manifold.contactPoints[i].p;

                renderer.DrawPoint(p1);

                Vec2 p2 = p1 + manifold.contactNormal * 0.15f;
                Vec2 t0 = p2 - manifold.contactNormal * 0.035f;
                Vec2 t1 = t0 + manifold.contactTangent * 0.02f;
                Vec2 t2 = t0 - manifold.contactTangent * 0.02f;

                renderer.DrawLine(p1, p2);
                renderer.DrawLine(p2, t1);
                renderer.DrawLine(p2, t2);
            }
        }
    }

    void UpdateShape1()
    {
        switch (item1)
        {
        case 0:
            shape1.reset(new Circle(0.5f));
            break;
        case 1:
            shape1.reset(new Capsule(0.9f, 0.4f));
            break;
        case 2:
            shape1.reset(new Polygon(1.0f));
            break;

        default:
            break;
        }
    }

    void UpdateShape2()
    {
        switch (item2)
        {
        case 0:
            shape2.reset(new Circle(0.4f));
            break;
        case 1:
            shape2.reset(new Capsule(0.8f, 0.3f));
            break;
        case 2:
            shape2.reset(new Polygon(0.8f));
            break;

        default:
            break;
        }
    }

    ~CollisionDetection()
    {
        options.show_contact_normal = false;
        options.show_contact_point = false;
    }

    static Demo* Create(Game& game)
    {
        return new CollisionDetection(game);
    }
};

static int index = register_demo("Collision detection", CollisionDetection::Create, 24);

} // namespace muli
