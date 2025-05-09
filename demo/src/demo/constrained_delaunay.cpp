#include "demo.h"
#include "window.h"

namespace muli
{

static bool constrained = true;

class ConstrainedDelaunay : public Demo
{
public:
    std::vector<Vec2> vertices;
    std::vector<std::vector<Vec2>> holes;

    std::vector<Polygon> triangles;

    size_t lastVertexCount = 0;

    std::vector<Vec2> currentHole;
    bool creatingHole = false;

    ConstrainedDelaunay(Game& game)
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

        if (!creatingHole && Input::IsKeyPressed(GLFW_KEY_LEFT_CONTROL))
        {
            creatingHole = true;
        }
        if (creatingHole && Input::IsKeyReleased(GLFW_KEY_LEFT_CONTROL))
        {
            holes.push_back(currentHole);

            creatingHole = false;
            currentHole.clear();
            lastVertexCount = 0;
        }

        if (!ImGui::GetIO().WantCaptureMouse)
        {
            if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                vertices.push_back(cursorPos);
            }

            if (creatingHole && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                currentHole.push_back(cursorPos);
                vertices.push_back(cursorPos);
            }
        }
    }

    void Step() override
    {
        size_t vertexCount = vertices.size();
        if (vertexCount != lastVertexCount)
        {
            if (constrained)
            {
                triangles = ComputeTriangles(vertices, holes);
            }
            else
            {
                triangles = ComputeTriangles(vertices);
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

        for (auto& hole : holes)
        {
            if (hole.size() > 1)
            {
                for (size_t i0 = hole.size() - 1, i1 = 0; i1 < hole.size(); i0 = i1, ++i1)
                {
                    renderer.DrawPoint(hole[i0], Vec4(1, 0, 0, 1));
                    if (constrained)
                    {
                        renderer.SetLineWidth(3);
                        renderer.DrawLine(hole[i0], hole[i1], Vec4(1, 1, 0, 1));
                    }
                }

                renderer.FlushLines();
                renderer.SetLineWidth(1);
            }
        }

        if (currentHole.size() > 1)
        {
            for (size_t i0 = currentHole.size() - 1, i1 = 0; i1 < currentHole.size(); i0 = i1, ++i1)
            {
                renderer.DrawPoint(currentHole[i0], Vec4(1, 0, 0, 1));
                if (constrained)
                {
                    renderer.SetLineWidth(3);
                    renderer.DrawLine(currentHole[i0], currentHole[i1], Vec4(1, 1, 0, 1));
                }
            }

            renderer.FlushLines();
            renderer.SetLineWidth(1);
        }

        for (const Polygon& p : triangles)
        {
            renderer.DrawShape(&p, identity, Renderer::DrawMode{});
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Delaunay", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            if (ImGui::Checkbox("Constrained", &constrained))
            {
                lastVertexCount = 0;
            }
        }
        ImGui::End();

        ImGui::SetNextWindowPos(
            { Window::Get()->GetWindowSize().x - 5, Window::Get()->GetWindowSize().y - 5 }, ImGuiCond_Always, { 1.0f, 1.0f }
        );
        ImGui::Begin(
            "DelaunayHelp", NULL,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
        );
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Left click to create normal vertex\nCtrl click to create constraint edges");
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new ConstrainedDelaunay(game);
    }
};

static int index = register_demo("Constrained delaunay", ConstrainedDelaunay::Create, 53);

} // namespace muli
