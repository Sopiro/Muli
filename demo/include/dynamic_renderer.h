#pragma once

#include "dynamic_shader.h"
#include "renderer.h"

namespace muli
{

static constexpr int32 max_vertex_count = 1024 * 3;

// Batch renderer
class DynamicRenderer final : public Renderer
{
public:
    DynamicRenderer();
    virtual ~DynamicRenderer();

    virtual void Render() override;

    void SetPointSize(float size) const;
    void SetLineWidth(float lineWidth) const;

    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetViewMatrix(const Mat4& viewMatrix);

    void DrawPoint(const Vec2& point, const Vec4& color = Vec4{ 0.0f, 0.0f, 0.0f, 1.0f });
    void DrawLine(const Vec2& point1,
                  const Vec2& point2,
                  const Vec4& color1 = Vec4{ 0.0f, 0.0f, 0.0f, 1.0f },
                  const Vec4& color2 = Vec4{ 0.0f, 0.0f, 0.0f, 1.0f });
    void DrawTriangle(
        const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec4& color1, const Vec4& color2, const Vec4& color3);
    void DrawTriangle(const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec4& color);
    void DrawShape(const Shape* shape, const Transform& tf, int32 colorIndex = 0);

private:
    friend class DynamicShader;

    void FlushAll();
    void FlushPoints();
    void FlushLines();
    void FlushTriangles();

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

inline void DynamicRenderer::SetPointSize(float size) const
{
    glPointSize(size);
}

inline void DynamicRenderer::SetLineWidth(float lineWidth) const
{
    glLineWidth(lineWidth);
}

inline void DynamicRenderer::Render()
{
    FlushAll();
}

inline void DynamicRenderer::FlushAll()
{
    if (triangleCount > 0) FlushTriangles();
    if (lineCount > 0) FlushLines();
    if (pointCount > 0) FlushPoints();
}

inline void DynamicRenderer::SetProjectionMatrix(const Mat4& _projMatrix)
{
    shader->Use();
    shader->SetProjectionMatrix(_projMatrix);
}

inline void DynamicRenderer::SetViewMatrix(const Mat4& _viewMatrix)
{
    shader->Use();
    shader->SetViewMatrix(_viewMatrix);
}

inline void DynamicRenderer::DrawPoint(const Vec2& point, const Vec4& color)
{
    if (pointCount == max_vertex_count)
    {
        FlushAll();
    }

    points[pointCount] = point;
    pointColors[pointCount] = color;
    ++pointCount;
}

inline void DynamicRenderer::DrawLine(const Vec2& point1, const Vec2& point2, const Vec4& color1, const Vec4& color2)
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

inline void DynamicRenderer::DrawTriangle(
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

inline void DynamicRenderer::DrawTriangle(const Vec2& point1, const Vec2& point2, const Vec2& point3, const Vec4& color)
{
    DrawTriangle(point1, point2, point3, color, color, color);
}

} // namespace muli