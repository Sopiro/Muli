#pragma once

#include "dynamic_shader.h"
#include "renderer.h"

namespace muli
{

static constexpr int32 max_vertex_count = 1024;

// Dynamic batch renderer
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
    void DrawShape(const Shape* shape, const Transform& tf);

private:
    friend class DynamicShader;

    void FlushPoints();
    void FlushLines();

    std::array<Vec2, max_vertex_count> points;
    std::array<Vec4, max_vertex_count> pointColors;
    int32 pointCount;

    std::array<Vec2, max_vertex_count> lines;
    std::array<Vec4, max_vertex_count> lineColors;
    int32 lineCount;

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
    FlushPoints();
    FlushLines();
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
        FlushPoints();
    }

    points[pointCount] = point;
    pointColors[pointCount] = color;
    ++pointCount;
}

inline void DynamicRenderer::DrawLine(const Vec2& point1, const Vec2& point2, const Vec4& color1, const Vec4& color2)
{
    if (lineCount == max_vertex_count)
    {
        FlushLines();
    }

    lines[lineCount] = point1;
    lineColors[lineCount] = color1;
    ++lineCount;
    lines[lineCount] = point2;
    lineColors[lineCount] = color2;
    ++lineCount;
}

} // namespace muli