#pragma once

#include "dynamic_shader.h"
#include "window.h"

namespace muli
{

static constexpr int32 max_vertex_count = 1024 * 3;
static constexpr Vec4 default_color{ 0.0f, 0.0f, 0.0f, 1.0f };

// Batch renderer
class Renderer final
{
public:
    Renderer();
    virtual ~Renderer();

    Renderer(const Renderer&) noexcept = delete;
    Renderer& operator=(const Renderer&) noexcept = delete;

    Renderer(Renderer&&) noexcept = delete;
    Renderer& operator=(Renderer&&) noexcept = delete;

    void SetPointSize(float size) const;
    void SetLineWidth(float lineWidth) const;

    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetViewMatrix(const Mat4& viewMatrix);

    void DrawPoint(const Vec2& point, const Vec4& color = default_color);
    void DrawLine(const Vec2& point1, const Vec2& point2, const Vec4& color1 = default_color, const Vec4& color2 = default_color);
    void DrawTriangle(
        const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec4& color1, const Vec4& color2, const Vec4& color3);
    void DrawTriangle(const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec4& color);
    void DrawShape(const Shape* shape, const Transform& tf, int32 colorIndex = 0);

    Vec2 Pick(Vec2 screenPos) const;

    void FlushAll();
    void FlushPoints();
    void FlushLines();
    void FlushTriangles();

private:
    friend class DynamicShader;

    std::array<Vec2, max_vertex_count> points;
    std::array<Vec4, max_vertex_count> pointColors;
    int32 pointCount;

    std::array<Vec2, max_vertex_count> lines;
    std::array<Vec4, max_vertex_count> lineColors;
    int32 lineCount;

    std::array<Vec2, max_vertex_count> triangles;
    std::array<Vec4, max_vertex_count> triangleColors;
    int32 triangleCount;

    std::unique_ptr<DynamicShader> shader;

    GLuint VAO;
    GLuint pVBO; // position buffer
    GLuint cVBO; // color buffer
};

inline void Renderer::SetPointSize(float size) const
{
    glPointSize(size);
}

inline void Renderer::SetLineWidth(float lineWidth) const
{
    glLineWidth(lineWidth);
}

inline void Renderer::FlushAll()
{
    if (triangleCount > 0) FlushTriangles();
    if (lineCount > 0) FlushLines();
    if (pointCount > 0) FlushPoints();
}

inline void Renderer::SetProjectionMatrix(const Mat4& _projMatrix)
{
    shader->Use();
    shader->SetProjectionMatrix(_projMatrix);
}

inline void Renderer::SetViewMatrix(const Mat4& _viewMatrix)
{
    shader->Use();
    shader->SetViewMatrix(_viewMatrix);
}

inline void Renderer::DrawPoint(const Vec2& point, const Vec4& color)
{
    if (pointCount == max_vertex_count)
    {
        FlushAll();
    }

    points[pointCount] = point;
    pointColors[pointCount] = color;
    ++pointCount;
}

inline void Renderer::DrawLine(const Vec2& point1, const Vec2& point2, const Vec4& color1, const Vec4& color2)
{
    if (lineCount == max_vertex_count)
    {
        FlushAll();
    }

    lines[lineCount] = point1;
    lineColors[lineCount] = color1;
    ++lineCount;
    lines[lineCount] = point2;
    lineColors[lineCount] = color2;
    ++lineCount;
}

inline void Renderer::DrawTriangle(
    const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec4& color1, const Vec4& color2, const Vec4& color3)
{
    if (triangleCount == max_vertex_count)
    {
        FlushAll();
    }

    triangles[triangleCount] = point1;
    triangleColors[triangleCount] = color1;
    ++triangleCount;
    triangles[triangleCount] = point2;
    triangleColors[triangleCount] = color2;
    ++triangleCount;
    triangles[triangleCount] = point3;
    triangleColors[triangleCount] = color3;
    ++triangleCount;
}

inline void Renderer::DrawTriangle(const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec4& color)
{
    DrawTriangle(point1, point2, point3, color, color, color);
}

// Viewport space -> NDC -> world spcae
inline Vec2 Renderer::Pick(Vec2 worldPos) const
{
    // Viewport space
    Vec2 windowSize = Window::Get().GetWindowSize();

    worldPos.y = windowSize.y - worldPos.y - 1;
    worldPos.x /= windowSize.x;
    worldPos.y /= windowSize.y;
    worldPos -= 0.5f;
    worldPos *= 2.0f;
    // NDC (-1 ~ 1)

    Mat4 invVP = (shader->projMatrix * shader->viewMatrix).GetInverse();

    // World space
    Vec4 invPos = invVP * Vec4{ worldPos.x, worldPos.y, 0.0f, 1.0f };

    return Vec2{ invPos.x, invPos.y };
}

} // namespace muli