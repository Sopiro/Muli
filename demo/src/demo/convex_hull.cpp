#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ConvexHull : public Demo
{
    static inline bool triangulation = false;

public:
    std::vector<Vec2> vertices;
    std::vector<Vec2> convexHull;
    std::vector<Polygon> triangles;

    size_t lastHull;

    ConvexHull(Game& game)
        : Demo(game)
    {
        float range = 2.0f;
        int32 count = 10;
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
        }
    }

    void Step() override
    {
        if (vertices.size() != lastHull)
        {
            convexHull = ComputeConvexHull(vertices);
            lastHull = convexHull.size();
            triangles = ComputeTriangles(vertices);
        }
    }

    void Render() override
    {
        for (Vec2& vertex : vertices)
        {
            renderer.DrawPoint(vertex);
        }
        if (!triangulation)
        {
            for (size_t i = 0; i < convexHull.size(); ++i)
            {
                Vec2& v0 = convexHull[i];
                Vec2& v1 = convexHull[(i + 1) % convexHull.size()];
                renderer.DrawLine(v0, v1);
            }
        }
        else
        {
            for (const Polygon& p : triangles)
            {
                renderer.DrawShape(&p, identity, Renderer::DrawMode{});
            }
        }
    }

    void UpdateUI() override
    {

        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("ConvexHull", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Checkbox("Triangulation", &triangulation);
            ImGui::Text("Removed vertices: %zu", vertices.size() - convexHull.size());
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new ConvexHull(game);
    }
};

static int index = register_demo("Convex hull", ConvexHull::Create, 21);

} // namespace muli
