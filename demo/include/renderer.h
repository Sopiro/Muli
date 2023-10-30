#pragma once

#include "batch_shader.h"
#include "window.h"

namespace muli
{

struct Vertex
{
    Vec2 point;
    Vec4 color;
};

// Batch renderer
class Renderer final : NonCopyable
{
    static inline bool initialized = false;

    static constexpr inline int32 circle_count = 12;
    static constexpr inline int32 quater_count = circle_count / 4;
    static inline std::array<Vec2, circle_count> unit_circle;

    static constexpr inline int32 color_count = 10;
    static inline std::array<Vec4, color_count> colors;

    static constexpr inline int32 max_vertex_count = 1024 * 3;

    static constexpr inline Vec4 color_transparent{ 0.0f, 0.0f, 0.0f, 0.0f };
    static constexpr inline Vec4 color_white{ 1.0f, 1.0f, 1.0f, 0.8f };
    static constexpr inline Vec4 default_color{ 0.0f, 0.0f, 0.0f, 1.0f };

public:
    Renderer();
    virtual ~Renderer();

    void SetPointSize(float size) const;
    void SetLineWidth(float lineWidth) const;

    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetViewMatrix(const Mat4& viewMatrix);

    void DrawPoint(const Vertex& v);
    void DrawPoint(const Vec2& p, const Vec4& color = default_color);
    void DrawLine(const Vertex& v1, const Vertex& v2);
    void DrawLine(const Vec2& p1, const Vec2& p2, const Vec4& color = default_color);
    void DrawTriangle(const Vertex& v1, const Vertex& v2, const Vertex& v3);
    void DrawTriangle(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec4& color = default_color);

    struct DrawMode
    {
        int32 colorIndex = -1;
        bool rounded = false;
        bool outline = true;
        bool fill = true;
    };

    void DrawShape(const Shape* shape, const Transform& tf, const DrawMode& mode);

    void FlushAll();
    void FlushPoints();
    void FlushLines();
    void FlushTriangles();

    Vec2 Pick(Vec2 screenPos) const;

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

inline void Renderer::DrawPoint(const Vertex& v)
{
    if (pointCount == max_vertex_count)
    {
        FlushAll();
    }

    points[pointCount] = v.point;
    pointColors[pointCount] = v.color;
    ++pointCount;
}

inline void Renderer::DrawPoint(const Vec2& p, const Vec4& color)
{
    if (pointCount == max_vertex_count)
    {
        FlushAll();
    }

    points[pointCount] = p;
    pointColors[pointCount] = color;
    ++pointCount;
}

inline void Renderer::DrawLine(const Vertex& v1, const Vertex& v2)
{
    if (lineCount == max_vertex_count)
    {
        FlushAll();
    }

    lines[lineCount] = v1.point;
    lineColors[lineCount] = v1.color;
    ++lineCount;
    lines[lineCount] = v2.point;
    lineColors[lineCount] = v2.color;
    ++lineCount;
}

inline void Renderer::DrawLine(const Vec2& p1, const Vec2& p2, const Vec4& color)
{
    if (lineCount == max_vertex_count)
    {
        FlushAll();
    }

    lines[lineCount] = p1;
    lineColors[lineCount] = color;
    ++lineCount;
    lines[lineCount] = p2;
    lineColors[lineCount] = color;
    ++lineCount;
}

inline void Renderer::DrawTriangle(const Vertex& v1, const Vertex& v2, const Vertex& v3)
{
    if (triangleCount == max_vertex_count)
    {
        FlushAll();
    }

    triangles[triangleCount] = v1.point;
    triangleColors[triangleCount] = v1.color;
    ++triangleCount;
    triangles[triangleCount] = v2.point;
    triangleColors[triangleCount] = v2.color;
    ++triangleCount;
    triangles[triangleCount] = v3.point;
    triangleColors[triangleCount] = v3.color;
    ++triangleCount;
}

inline void Renderer::DrawTriangle(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec4& color)
{
    if (triangleCount == max_vertex_count)
    {
        FlushAll();
    }

    triangles[triangleCount] = p1;
    triangleColors[triangleCount] = color;
    ++triangleCount;
    triangles[triangleCount] = p2;
    triangleColors[triangleCount] = color;
    ++triangleCount;
    triangles[triangleCount] = p3;
    triangleColors[triangleCount] = color;
    ++triangleCount;
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