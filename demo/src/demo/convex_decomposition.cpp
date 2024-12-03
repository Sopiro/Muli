#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ConvexDecomposition : public Demo
{
public:
    std::vector<Vec2> outline;
    std::vector<std::vector<Vec2>> holes;

    std::vector<Polygon> triangles;
    std::vector<Vec2> currentHole;

    bool creatingHole = false;

    ConvexDecomposition(Game& game)
        : Demo(game)
    {
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
        }

        if (!ImGui::GetIO().WantCaptureMouse)
        {
            if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                if (creatingHole)
                {
                    currentHole.push_back(cursorPos);
                }
                else
                {
                    outline.push_back(cursorPos);
                }
            }
        }
    }

    void Step() override
    {
        if (Input::IsKeyPressed(GLFW_KEY_SPACE))
        {
            triangles = ComputeDecomposition(outline, holes);
        }
    }

    void Render() override
    {
        for (auto& hole : holes)
        {
            renderer.SetLineWidth(3);
            for (size_t i0 = hole.size() - 1, i1 = 0; i1 < hole.size(); i0 = i1, ++i1)
            {
                renderer.DrawLine(hole[i0], hole[i1], Vec4(1, 1, 0, 1));
            }
            renderer.FlushLines();
            renderer.SetLineWidth(1);
            for (size_t i0 = hole.size() - 1, i1 = 0; i1 < hole.size(); i0 = i1, ++i1)
            {
                renderer.DrawPoint(hole[i0], Vec4(1, 0, 0, 1));
                renderer.DrawLine(hole[i0], hole[i1]);
            }
            renderer.FlushLines();
        }

        if (currentHole.size() > 0)
        {
            renderer.SetLineWidth(3);
            for (size_t i0 = currentHole.size() - 1, i1 = 0; i1 < currentHole.size(); i0 = i1, ++i1)
            {
                renderer.DrawLine(currentHole[i0], currentHole[i1], Vec4(1, 1, 0, 1));
            }
            renderer.FlushLines();
            renderer.SetLineWidth(1);
            for (size_t i0 = currentHole.size() - 1, i1 = 0; i1 < currentHole.size(); i0 = i1, ++i1)
            {
                renderer.DrawPoint(currentHole[i0], Vec4(1, 0, 0, 1));
                renderer.DrawLine(currentHole[i0], currentHole[i1]);
            }
            renderer.FlushLines();
        }

        renderer.SetLineWidth(3);
        for (int32 i0 = outline.size() - 1, i1 = 0; i1 < outline.size(); i0 = i1, ++i1)
        {
            renderer.DrawLine(outline[i0], outline[i1], Vec4(0, 1, 0.2f, 1));
        }
        renderer.FlushLines();
        renderer.SetLineWidth(1);
        for (int32 i0 = outline.size() - 1, i1 = 0; i1 < outline.size(); i0 = i1, ++i1)
        {
            renderer.DrawPoint(outline[i0], Vec4(1, 0, 0, 1));
            renderer.DrawLine(outline[i0], outline[i1]);
        }
        renderer.FlushLines();

        for (const Polygon& p : triangles)
        {
            renderer.DrawShape(&p, identity, Renderer::DrawMode{});
        }

        ImGui::SetNextWindowPos(
            { Window::Get().GetWindowSize().x - 5, Window::Get().GetWindowSize().y - 5 }, ImGuiCond_Always, { 1.0f, 1.0f }
        );
        ImGui::Begin(
            "DecompositionHelp", NULL,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
        );
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Left click to create outline\nPress space to run decomposition");
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new ConvexDecomposition(game);
    }
};

static int index = register_demo("Convex decomposition", ConvexDecomposition::Create, 54);

} // namespace muli
