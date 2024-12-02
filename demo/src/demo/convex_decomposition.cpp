#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ConvexDecomposition : public Demo
{
public:
    std::vector<Vec2> outline;

    std::vector<Polygon> triangles;

    ConvexDecomposition(Game& game)
        : Demo(game)
    {
    }

    void UpdateInput() override
    {
        FindTargetBody();
        EnableCameraControl();

        if (!ImGui::GetIO().WantCaptureMouse)
        {
            if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                outline.push_back(cursorPos);
            }
        }
    }

    void Step() override
    {
        if (Input::IsKeyPressed(GLFW_KEY_SPACE))
        {
            triangles = ComputeDecomposition(outline);
        }
    }

    void Render() override
    {
        for (int32 i0 = outline.size() - 1, i1 = 0; i1 < outline.size(); i0 = i1, ++i1)
        {
            renderer.DrawPoint(outline[i1]);
            renderer.DrawLine(outline[i0], outline[i1]);
        }

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
