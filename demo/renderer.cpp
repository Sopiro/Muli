#include "renderer.h"
#include "util.h"

namespace muli
{

namespace
{

constexpr int32 sdf_quad_vertex_count = 6;

constexpr std::array<Vec2, sdf_quad_vertex_count> sdf_quad_vertices = {
    Vec2{ 0.0f, 0.0f }, Vec2{ 1.0f, 0.0f }, Vec2{ 1.0f, 1.0f }, Vec2{ 0.0f, 0.0f }, Vec2{ 1.0f, 1.0f }, Vec2{ 0.0f, 1.0f },
};

float GetSdfPadding(const Vec2& pixelWorldSize)
{
    return 2.0f * Max(pixelWorldSize.x, pixelWorldSize.y);
}

void SetVec4InstanceAttrib(GLuint index, GLsizei stride, size_t offset)
{
    glVertexAttribPointer(index, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(offset));
    glEnableVertexAttribArray(index);
    glVertexAttribDivisor(index, 1);
}

void SetIVec4InstanceAttrib(GLuint index, GLsizei stride, size_t offset)
{
    glVertexAttribIPointer(index, 4, GL_INT, stride, reinterpret_cast<const void*>(offset));
    glEnableVertexAttribArray(index);
    glVertexAttribDivisor(index, 1);
}

} // namespace

Renderer::Renderer()
    : pointCount{ 0 }
    , lineCount{ 0 }
    , triangleCount{ 0 }
{
    points.resize(max_vertex_count);
    pointColors.resize(max_vertex_count);
    lines.resize(max_vertex_count);
    lineColors.resize(max_vertex_count);
    triangles.resize(max_vertex_count);
    triangleColors.resize(max_vertex_count);

    shader = BatchShader::Create();
    shapeShader = ShapeSdfShader::Create();

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &pVBO);
    glGenBuffers(1, &cVBO);

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * max_vertex_count, nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2), 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, cVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * max_vertex_count, nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vec4), 0);
        glEnableVertexAttribArray(1);
    }
    glBindVertexArray(0);

    glGenVertexArrays(1, &sdfVAO);
    glGenBuffers(1, &sdfQuadVBO);
    glGenBuffers(1, &sdfInstanceVBO);
    glGenTextures(1, &sdfPolygonTexture);

    glBindVertexArray(sdfVAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, sdfQuadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * sdf_quad_vertex_count, sdf_quad_vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2), 0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, sdfInstanceVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(ShapeInstanceData) * max_shape_batch_count, nullptr, GL_DYNAMIC_DRAW);

        constexpr GLsizei stride = sizeof(ShapeInstanceData);
        SetVec4InstanceAttrib(1, stride, offsetof(ShapeInstanceData, localBounds));
        SetVec4InstanceAttrib(2, stride, offsetof(ShapeInstanceData, transform0));
        SetVec4InstanceAttrib(3, stride, offsetof(ShapeInstanceData, transform1));
        SetVec4InstanceAttrib(4, stride, offsetof(ShapeInstanceData, capsule));
        SetVec4InstanceAttrib(5, stride, offsetof(ShapeInstanceData, fillColor));
        SetVec4InstanceAttrib(6, stride, offsetof(ShapeInstanceData, outlineColor));
        SetVec4InstanceAttrib(7, stride, offsetof(ShapeInstanceData, scalarData));
        SetIVec4InstanceAttrib(8, stride, offsetof(ShapeInstanceData, shapeType));
    }
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, sdfPolygonTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, max_sdf_polygon_vertices, max_shape_batch_count, 0, GL_RGBA, GL_FLOAT, nullptr);
    glBindTexture(GL_TEXTURE_2D, 0);

    shapeShader->Use();
    shapeShader->SetPolygonVertexTextureUnit(0);
    shapeShader->SetPixelWorldSize(pixelWorldSize);

    if (initialized == false)
    {
        constexpr float stride = 360.0f / color_count;
        for (int32 i = 0; i < color_count; ++i)
        {
            Vec3 rgb = HSLtoRGB({ i * stride / 360.0f, 1.0f, 0.8f });
            colors[i].Set(rgb.x, rgb.y, rgb.z, 0.85f);
        }

        initialized = true;
    }
}

Renderer::~Renderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &pVBO);
    glDeleteBuffers(1, &cVBO);

    glDeleteVertexArrays(1, &sdfVAO);
    glDeleteBuffers(1, &sdfQuadVBO);
    glDeleteBuffers(1, &sdfInstanceVBO);
    glDeleteTextures(1, &sdfPolygonTexture);
}

void Renderer::EnsurePrimitiveCapacity(std::vector<Vec2>& vertices, std::vector<Vec4>& colors, int32 requiredCount)
{
    if (requiredCount <= (int32)vertices.size())
    {
        return;
    }

    int32 newCapacity = Max<int32>((int32)vertices.size() * 2, requiredCount);
    vertices.resize(newCapacity);
    colors.resize(newCapacity);
}

void Renderer::EnsureBatchBufferCapacity(int32 requiredCount)
{
    if (requiredCount <= batchBufferCapacity)
    {
        return;
    }

    batchBufferCapacity = Max(batchBufferCapacity * 2, requiredCount);

    glBindBuffer(GL_ARRAY_BUFFER, pVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * batchBufferCapacity, nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, cVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * batchBufferCapacity, nullptr, GL_DYNAMIC_DRAW);
}

void Renderer::QueueShapeInstance(ShapeInstanceData instance, const Vec2* polygonVertices)
{
    if (shapeCount == max_shape_batch_count)
    {
        FlushShapes();
    }

    if (polygonVertices && instance.vertexCount > 0)
    {
        if (packedPolygonVertexCount + instance.vertexCount > max_packed_polygon_vertex_count)
        {
            FlushShapes();
        }

        instance.vertexOffset = packedPolygonVertexCount;
        const int32 base = packedPolygonVertexCount;
        for (int32 i = 0; i < instance.vertexCount; ++i)
        {
            const Vec2& vertex = polygonVertices[i];
            polygonVertexTexels[base + i] = Vec4{ vertex.x, vertex.y, 0.0f, 0.0f };
        }

        packedPolygonVertexCount += instance.vertexCount;
    }
    else
    {
        instance.vertexOffset = 0;
    }

    shapeInstances[shapeCount] = instance;
    ++shapeCount;
}

void Renderer::DrawShape(const Shape* shape, const Transform& tf, const DrawMode& mode)
{
    if (mode.fill == false && mode.outline == false)
    {
        return;
    }

    const Vec4& fillColor = mode.colorIndex < 0 ? default_white : colors[mode.colorIndex % color_count];
    const float padding = GetSdfPadding(pixelWorldSize);

    ShapeInstanceData instance{};
    instance.transform0 = Vec4{ tf.position.x, tf.position.y, tf.rotation.c, tf.rotation.s };
    instance.transform1 = Vec4{ -tf.rotation.s, tf.rotation.c, 0.0f, 0.0f };
    instance.fillColor = fillColor;
    instance.outlineColor = default_black;
    instance.shapeType = shape->GetType();
    instance.flags = (mode.fill ? shape_flag_fill : 0) | (mode.outline ? shape_flag_outline : 0);

    const Vec2* polygonVertices = nullptr;

    switch (shape->GetType())
    {
    case Shape::Type::circle:
    {
        const Circle* c = (const Circle*)shape;
        const Vec2 center = c->GetCenter();
        const float radius = c->GetRadius();
        const Vec2 pad{ radius + padding };

        instance.localBounds = Vec4{ center.x - pad.x, center.y - pad.y, center.x + pad.x, center.y + pad.y };
        instance.transform1.z = center.x;
        instance.transform1.w = center.y;
        instance.scalarData.x = radius;

        QueueShapeInstance(instance, nullptr);

        if (mode.outline)
        {
            DrawLine(Mul(tf, center), Mul(tf, center + Vec2{ radius, 0.0f }), default_black);
        }
    }
    break;
    case Shape::Type::capsule:
    {
        const Capsule* c = (const Capsule*)shape;
        const Vec2 va = c->GetVertexA();
        const Vec2 vb = c->GetVertexB();
        const float radius = c->GetRadius();
        const Vec2 pad{ radius + padding };
        const Vec2 min = Min(va, vb) - pad;
        const Vec2 max = Max(va, vb) + pad;

        instance.localBounds = Vec4{ min.x, min.y, max.x, max.y };
        instance.capsule = Vec4{ va.x, va.y, vb.x, vb.y };
        instance.scalarData.x = radius;

        QueueShapeInstance(instance, nullptr);
    }
    break;
    case Shape::Type::polygon:
    {
        const Polygon* p = (const Polygon*)shape;
        const int32 vertexCount = p->GetVertexCount();

        if (vertexCount > max_sdf_polygon_vertices)
        {
            std::cout << "ShapeSdfRenderer: polygon " << static_cast<const void*>(shape) << " exceeds "
                      << max_sdf_polygon_vertices << " vertices (" << vertexCount << "); skipping render\n";
            return;
        }

        polygonVertices = p->GetVertices();

        float radius = mode.rounded ? p->GetRadius() : 0.0f;
        Vec2 min = polygonVertices[0];
        Vec2 max = polygonVertices[0];
        for (int32 i = 1; i < vertexCount; ++i)
        {
            min = Min(min, polygonVertices[i]);
            max = Max(max, polygonVertices[i]);
        }

        const Vec2 pad{ radius + padding };
        min -= pad;
        max += pad;

        instance.localBounds = Vec4{ min.x, min.y, max.x, max.y };
        instance.scalarData.x = radius;
        instance.vertexCount = vertexCount;

        QueueShapeInstance(instance, polygonVertices);
    }
    break;
    default:
        MuliAssert(false);
        break;
    }
}

void Renderer::FlushShapes()
{
    if (shapeCount == 0)
    {
        return;
    }

    shapeShader->Use();
    shapeShader->SetPixelWorldSize(pixelWorldSize);

    glBindVertexArray(sdfVAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, sdfInstanceVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(ShapeInstanceData) * shapeCount, shapeInstances.data());

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, sdfPolygonTexture);

        if (packedPolygonVertexCount > 0)
        {
            const int32 uploadHeight = (packedPolygonVertexCount + max_sdf_polygon_vertices - 1) / max_sdf_polygon_vertices;
            const int32 uploadTexelCount = uploadHeight * max_sdf_polygon_vertices;

            for (int32 i = packedPolygonVertexCount; i < uploadTexelCount; ++i)
            {
                polygonVertexTexels[i] = Vec4::zero;
            }

            glTexSubImage2D(
                GL_TEXTURE_2D, 0, 0, 0, max_sdf_polygon_vertices, uploadHeight, GL_RGBA, GL_FLOAT, polygonVertexTexels.data()
            );
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArraysInstanced(GL_TRIANGLES, 0, sdf_quad_vertex_count, shapeCount);
        glDisable(GL_BLEND);

        glBindTexture(GL_TEXTURE_2D, 0);
    }
    glBindVertexArray(0);

    shapeCount = 0;
    packedPolygonVertexCount = 0;
}

void Renderer::FlushPoints()
{
    shader->Use();

    glBindVertexArray(VAO);
    {
        EnsureBatchBufferCapacity(pointCount);

        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * pointCount, points.data());

        glBindBuffer(GL_ARRAY_BUFFER, cVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4) * pointCount, pointColors.data());

        glDrawArrays(GL_POINTS, 0, pointCount);
    }
    glBindVertexArray(0);

    pointCount = 0;
}

void Renderer::FlushLines()
{
    shader->Use();

    glBindVertexArray(VAO);
    {
        EnsureBatchBufferCapacity(lineCount);

        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * lineCount, lines.data());

        glBindBuffer(GL_ARRAY_BUFFER, cVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4) * lineCount, lineColors.data());

        glDrawArrays(GL_LINES, 0, lineCount);
    }
    glBindVertexArray(0);

    lineCount = 0;
}

void Renderer::FlushTriangles()
{
    shader->Use();

    glBindVertexArray(VAO);
    {
        EnsureBatchBufferCapacity(triangleCount);

        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * triangleCount, triangles.data());

        glBindBuffer(GL_ARRAY_BUFFER, cVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4) * triangleCount, triangleColors.data());

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDrawArrays(GL_TRIANGLES, 0, triangleCount);
        glDisable(GL_BLEND);
    }
    glBindVertexArray(0);

    triangleCount = 0;
}

} // namespace muli
