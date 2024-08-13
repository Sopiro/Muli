#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ConstrainedDelauney : public Demo
{
    static inline bool constrained = true;

public:
    std::vector<Vec2> vertices;
    std::vector<Vec2> constraints;

    std::vector<Polygon> triangles;

    size_t lastVertexCount = 0;

    ConstrainedDelauney(Game& game)
        : Demo(game)
    {
        float range = 3.0f;
        int32 count = 0;
        for (int32 i = 0; i < count; ++i)
        {
            vertices.push_back(RandVec2(Vec2{ -range }, Vec2{ range }));
        }

        camera.position.SetZero();
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableCameraControl();

        if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
        {
            if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                vertices.push_back(cursorPos);
            }

            if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_RIGHT))
            {
                constraints.push_back(cursorPos);
            }
        }
    }

    void Step() override
    {
        size_t vertexCount = vertices.size() + constraints.size();
        if (vertexCount != lastVertexCount)
        {
            if (constrained)
            {
                triangles = ComputeTriangles(vertices, constraints);
            }
            else
            {
                std::vector<Vec2> v(vertices.begin(), vertices.end());
                v.insert(v.end(), constraints.begin(), constraints.end());
                triangles = ComputeTriangles(v);
            }

            lastVertexCount = vertexCount;
        }
    }

    void Render() override
    {
        for (Vec2& vertex : vertices)
        {
            renderer.DrawPoint(vertex);
        }

        if (constraints.size() > 0)
        {
            for (size_t i = 0; i < constraints.size() - 1; ++i)
            {
                renderer.DrawPoint(constraints[i], Vec4(1, 0, 0, 1));
                if (constrained)
                {
                    renderer.SetLineWidth(3);
                    renderer.DrawLine(constraints[i], constraints[i + 1], Vec4(1, 1, 0, 1));
                    renderer.FlushLines();
                    renderer.SetLineWidth(1);
                }
            }
            renderer.DrawPoint(constraints.back(), Vec4(1, 0, 0, 1));
        }

        for (const Polygon& p : triangles)
        {
            renderer.DrawShape(&p, identity, Renderer::DrawMode{ .fill = false });
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Delauney", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::Checkbox("Constrained", &constrained))
            {
                lastVertexCount = 0;
            }
        }
        ImGui::End();

        ImGui::SetNextWindowPos(
            { Window::Get().GetWindowSize().x - 5, Window::Get().GetWindowSize().y - 5 }, ImGuiCond_Always, { 1.0f, 1.0f }
        );
        ImGui::Begin(
            "DelauneyHelp", NULL,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
        );
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Left click to create normal vertex\nRight click to create constraint vertex");
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new ConstrainedDelauney(game);
    }
};

static int index = register_demo("Constrained delauney", ConstrainedDelauney::Create, 53);

} // namespace muli
