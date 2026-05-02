#include "demo.h"

namespace muli
{

static const int32 num_drawmodes = 3;
static const char* drawmodes[num_drawmodes] = { "Points", "Lines", "Points + Lines" };

static const int32 num_splines = 3;
static const char* splines[num_splines] = { "Catmull-Rom Spline", "Cardinal Spline", "Uniform B-Spline" };
static int32 drawIndex = 2;
static int32 splineIndex = 0;

class Splines : public Demo
{
public:
    Splines(Game& game)
        : Demo(game)
    {
    }

    bool mirrorEndPoints = true;

    std::vector<Vec2> points;
    int32 pointSamples = 10;

    float tension = 0;

    virtual void UpdateInput()
    {
        FindTargetBody();
        EnableCameraControl();

        if (!ImGui::GetIO().WantCaptureMouse && Input::IsMousePressed(GLFW_MOUSE_BUTTON_LEFT))
        {
            if (points.size() == 0) points.push_back(cursorPos);
            points.push_back(cursorPos);
        }
    }

    virtual void Step() {}

    virtual void Render()
    {
        if (points.size() > 2)
        {
            points.push_back(points.back());
            if (mirrorEndPoints)
            {
                Vec2 d = points[1] - points[2];
                points[0] += d;

                d = points[points.size() - 2] - points[points.size() - 3];
                points[points.size() - 1] += d;
            }

            for (int j = 0; j < points.size() - 3; ++j)
            {
                Vec2 t0;
                for (int i = 0; i <= pointSamples; ++i)
                {
                    float t = i / float(pointSamples);

                    Vec2 t1;
                    switch (splineIndex)
                    {
                    case 0:
                        t1 = SplineCatmullRom(points[j + 0], points[j + 1], points[j + 2], points[j + 3], t);
                        break;
                    case 1:
                        t1 = SplineCardinal(points[j + 0], points[j + 1], points[j + 2], points[j + 3], t, tension);
                        break;
                    case 2:
                        t1 = SplineUniformBasis(points[j + 0], points[j + 1], points[j + 2], points[j + 3], t);
                        break;
                    default:
                        break;
                    }
                    if (drawIndex != 1)
                    {
                        renderer.DrawPoint(t1);
                    }

                    if (drawIndex > 0)
                    {
                        if (i != 0) renderer.DrawLine(t0, t1);
                        t0 = t1;
                    }
                }
            }

            if (mirrorEndPoints)
            {
                points[0] = points[1];
            }
            points.pop_back();
        }

        if (drawIndex != 1)
        {
            for (int i = 0; i < points.size(); ++i)
            {
                renderer.DrawPoint(points[i], Vec4(1, 0, 0, 1));
            }
        }
    }

    virtual void UpdateUI()
    {
        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Always, { 1.0f, 0.0f });

        if (ImGui::Begin("Splines", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Checkbox("Mirror endpoints", &mirrorEndPoints);
            ImGui::Text("Point samples");
            ImGui::SliderInt("##Point samples", &pointSamples, 1, 100);

            ImGui::Text("Draw mode");
            ImGui::Combo("##Draw mode", &drawIndex, drawmodes, num_drawmodes);
            ImGui::Text("Spline");
            ImGui::Combo("##Spline", &splineIndex, splines, num_splines);

            ImGui::Separator();
            if (splineIndex == 1)
            {
                ImGui::Text("tension");
                ImGui::SliderFloat("##tension", &tension, 0, 1);
            }
        }
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new Splines(game);
    }
};

static int index = register_demo("Splines", Splines::Create, 54);

} // namespace muli
