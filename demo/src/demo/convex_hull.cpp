#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ConvexHull : public Demo
{
public:
    std::vector<Vec2> vertices;
    std::vector<Vec2> convexHull;

    size_t lastHull;

    ConvexHull(Game& game)
        : Demo(game)
    {
        float range = 2.0f;
        uint32 count = 10;

        for (uint32 i = 0; i < count; ++i)
        {
            vertices.push_back(LinearRand(Vec2{ -range }, Vec2{ range }));
        }

        camera.position.SetZero();
    }

    void UpdateInput() override
    {
        ComputeProperty();
        EnableCameraControl();

        if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
        {
            if (Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                vertices.push_back(mpos);
            }
        }
    }

    void Step() override
    {
        if (vertices.size() != lastHull)
        {
            convexHull = ComputeConvexHull(vertices);
            lastHull = vertices.size();
        }
    }

    void Render() override
    {
        std::vector<Vec2>& pl = game.GetPointList();
        std::vector<Vec2>& ll = game.GetLineList();

        pl = vertices;

        for (uint32 i = 0; i < convexHull.size(); ++i)
        {
            Vec2& v0 = convexHull[i];
            Vec2& v1 = convexHull[(i + 1) % convexHull.size()];
            ll.push_back(v0);
            ll.push_back(v1);
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });
        ImGui::Begin("Overlay", NULL,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground);
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "Removed vertices: %ld", vertices.size() - convexHull.size());
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new ConvexHull(game);
    }
};

DemoFrame convex_hull{ "Convex hull", ConvexHull::Create };

} // namespace muli
