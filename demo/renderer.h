#pragma once

#include "batch_shader.h"
#include "shape_sdf_shader.h"
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

    static constexpr inline int32 max_sdf_polygon_vertices = 64;
    static constexpr inline int32 max_shape_batch_count = 1024;
    static constexpr inline int32 max_packed_polygon_vertex_count = max_shape_batch_count * max_sdf_polygon_vertices;

    static constexpr inline int32 color_count = 10;
    static inline std::array<Vec4, color_count> colors;

    static constexpr inline int32 max_vertex_count = 1024 * 3;

    static constexpr inline int32 shape_flag_fill = 1 << 0;
    static constexpr inline int32 shape_flag_outline = 1 << 1;

    static constexpr inline Vec4 default_white{ 1.0f, 1.0f, 1.0f, 0.8f };
    static constexpr inline Vec4 default_black{ 0.0f, 0.0f, 0.0f, 0.9f };

public:
    Renderer();
    virtual ~Renderer();

    void SetPointSize(float size) const;
    void SetLineWidth(float lineWidth) const;

    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetViewMatrix(const Mat4& viewMatrix);
    void SetPixelWorldSize(const Vec2& pixelWorldSize);

    void DrawPoint(const Vertex& v);
    void DrawPoint(const Vec2& p, const Vec4& color = default_black);
    void DrawLine(const Vertex& v1, const Vertex& v2);
    void DrawLine(const Vec2& p1, const Vec2& p2, const Vec4& color = default_black);
    void DrawTriangle(const Vertex& v1, const Vertex& v2, const Vertex& v3);
    void DrawTriangle(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec4& color = default_black);
    void DrawAABB(const AABB& aabb);

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
    friend class BatchShader;

    struct ShapeInstanceData
    {
        Vec4 localBounds;
        Vec4 transform0;
        Vec4 transform1;
        Vec4 capsule;
        Vec4 fillColor;
        Vec4 outlineColor;
        Vec4 scalarData;
        int32 shapeType;
        int32 flags;
        int32 vertexCount;
        int32 vertexOffset;
    };

    void FlushShapes();
    void QueueShapeInstance(ShapeInstanceData instance, const Vec2* polygonVertices);
    void EnsurePrimitiveCapacity(std::vector<Vec2>& vertices, std::vector<Vec4>& colors, int32 requiredCount);
    void EnsureBatchBufferCapacity(int32 requiredCount);

    std::vector<Vec2> points;
    std::vector<Vec4> pointColors;
    int32 pointCount;

    std::vector<Vec2> lines;
    std::vector<Vec4> lineColors;
    int32 lineCount;

    std::vector<Vec2> triangles;
    std::vector<Vec4> triangleColors;
    int32 triangleCount;

    std::array<ShapeInstanceData, max_shape_batch_count> shapeInstances;
    std::array<Vec4, max_packed_polygon_vertex_count> polygonVertexTexels;
    int32 shapeCount = 0;
    int32 packedPolygonVertexCount = 0;

    std::unique_ptr<BatchShader> shader;
    std::unique_ptr<ShapeSdfShader> shapeShader;

    Vec2 pixelWorldSize{ 0.01f };

    GLuint VAO;
    GLuint pVBO;
    GLuint cVBO;
    int32 batchBufferCapacity = max_vertex_count;

    GLuint sdfVAO = 0;
    GLuint sdfQuadVBO = 0;
    GLuint sdfInstanceVBO = 0;
    GLuint sdfPolygonTexture = 0;
};

inline void Renderer::SetPointSize(float size) const
{
#ifndef __EMSCRIPTEN__
    glPointSize(size);
#else
    MuliNotUsed(size);
#endif
}

inline void Renderer::SetLineWidth(float lineWidth) const
{
    glLineWidth(lineWidth);
}

inline void Renderer::FlushAll()
{
    if (triangleCount > 0) FlushTriangles();
    if (shapeCount > 0) FlushShapes();
    if (lineCount > 0) FlushLines();
    if (pointCount > 0) FlushPoints();
}

inline void Renderer::SetProjectionMatrix(const Mat4& newProjMatrix)
{
    shader->Use();
    shader->SetProjectionMatrix(newProjMatrix);

    shapeShader->Use();
    shapeShader->SetProjectionMatrix(newProjMatrix);
}

inline void Renderer::SetViewMatrix(const Mat4& newViewMatrix)
{
    shader->Use();
    shader->SetViewMatrix(newViewMatrix);

    shapeShader->Use();
    shapeShader->SetViewMatrix(newViewMatrix);
}

inline void Renderer::SetPixelWorldSize(const Vec2& newPixelWorldSize)
{
    pixelWorldSize = newPixelWorldSize;
}

inline void Renderer::DrawPoint(const Vertex& v)
{
    EnsurePrimitiveCapacity(points, pointColors, pointCount + 1);

    points[pointCount] = v.point;
    pointColors[pointCount] = v.color;
    ++pointCount;
}

inline void Renderer::DrawPoint(const Vec2& p, const Vec4& color)
{
    EnsurePrimitiveCapacity(points, pointColors, pointCount + 1);

    points[pointCount] = p;
    pointColors[pointCount] = color;
    ++pointCount;
}

inline void Renderer::DrawLine(const Vertex& v1, const Vertex& v2)
{
    EnsurePrimitiveCapacity(lines, lineColors, lineCount + 2);

    lines[lineCount] = v1.point;
    lineColors[lineCount] = v1.color;
    ++lineCount;
    lines[lineCount] = v2.point;
    lineColors[lineCount] = v2.color;
    ++lineCount;
}

inline void Renderer::DrawLine(const Vec2& p1, const Vec2& p2, const Vec4& color)
{
    EnsurePrimitiveCapacity(lines, lineColors, lineCount + 2);

    lines[lineCount] = p1;
    lineColors[lineCount] = color;
    ++lineCount;
    lines[lineCount] = p2;
    lineColors[lineCount] = color;
    ++lineCount;
}

inline void Renderer::DrawTriangle(const Vertex& v1, const Vertex& v2, const Vertex& v3)
{
    EnsurePrimitiveCapacity(triangles, triangleColors, triangleCount + 3);

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
    EnsurePrimitiveCapacity(triangles, triangleColors, triangleCount + 3);

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

inline void Renderer::DrawAABB(const AABB& aabb)
{
    Vec2 br{ aabb.max.x, aabb.min.y };
    Vec2 tl{ aabb.min.x, aabb.max.y };
    DrawLine(aabb.min, br);
    DrawLine(br, aabb.max);
    DrawLine(aabb.max, tl);
    DrawLine(tl, aabb.min);
}

// Viewport space -> NDC -> world spcae
inline Vec2 Renderer::Pick(Vec2 worldPos) const
{
    // Viewport space
    Vec2 windowSize = Window::Get()->GetWindowSize();

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
