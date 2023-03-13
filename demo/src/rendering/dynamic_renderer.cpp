#include "dynamic_renderer.h"

namespace muli
{

static bool initialized = false;
static constexpr int32 circle_count = 12;
static constexpr int32 quater_count = circle_count / 4;
static std::array<Vec2, circle_count> unit_circle;

DynamicRenderer::DynamicRenderer()
    : pointCount{ 0 }
    , lineCount{ 0 }
{
    shader = DynamicShader::Create();

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

    if (initialized == false)
    {
        muliAssert(circle_count % 4 == 0);

        constexpr float angle = pi * 2.0f / circle_count;
        for (int32 i = 0; i < circle_count; ++i)
        {
            float currentAngle = angle * i;

            unit_circle[i].Set(Cos(currentAngle), Sin(currentAngle));
        }

        initialized = true;
    }
}

DynamicRenderer::~DynamicRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &pVBO);
    glDeleteBuffers(1, &cVBO);
}

void DynamicRenderer::DrawShape(const Shape* shape, const Transform& tf)
{
    switch (shape->GetType())
    {
    case Shape::Type::circle:
    {
        const Circle* c = static_cast<const Circle*>(shape);
        Vec2 localCenter = c->GetCenter();
        float radius = c->GetRadius();

        int32 i0 = circle_count - 1;
        int32 i1 = 0;
        Vec2 v0 = Mul(tf, localCenter + unit_circle[i0] * radius);
        Vec2 v1 = Mul(tf, localCenter + unit_circle[i1] * radius);

        DrawLine(Mul(tf, localCenter), v1);
        DrawLine(v0, v1);

        i0 = i1;
        v0 = v1;

        for (i1 = 1; i1 < circle_count; ++i1)
        {
            v1 = Mul(tf, localCenter + unit_circle[i1] * radius);
            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }
    }
    break;
    case Shape::Type::capsule:
    {
        const Capsule* c = static_cast<const Capsule*>(shape);

        Vec2 va = c->GetVertexA();
        Vec2 vb = c->GetVertexB();
        Vec2 normal = Cross(1.0f, vb - va).Normalized();

        Vec2 dir = vb - va;
        float angleOffset = AngleBetween(Vec2{ 1.0f, 0.0 }, dir);

        Rotation rot{ angleOffset };

        float radius = c->GetRadius();

        int32 i0 = 0;
        Vec2 v0 = Mul(tf, vb + Mul(rot, unit_circle[i0] * radius));
        Vec2 v1;

        for (int32 i1 = 1; i0 < quater_count; ++i1)
        {
            v1 = Mul(tf, vb + Mul(rot, unit_circle[i1] * radius));

            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }

        for (int32 i1 = quater_count; i0 < quater_count * 3; ++i1)
        {
            v1 = Mul(tf, va + Mul(rot, unit_circle[i1] * radius));

            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }

        for (int32 i1 = quater_count * 3; i0 < quater_count * 4 - 1; ++i1)
        {
            v1 = Mul(tf, vb + Mul(rot, unit_circle[i1] * radius));

            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }

        v0 = Mul(tf, vb + Mul(rot, unit_circle[0] * radius));

        DrawLine(v1, v0);
    }
    break;
    case Shape::Type::polygon:
    {
        const Polygon* p = static_cast<const Polygon*>(shape);
        float radius = p->GetRadius();

        const Vec2* vertices = p->GetVertices();
        int32 vertexCount = p->GetVertexCount();

        int32 i0 = vertexCount - 1;
        Vec2 v0 = Mul(tf, vertices[i0]);
        for (int32 i1 = 0; i1 < vertexCount; ++i1)
        {
            Vec2 v1 = Mul(tf, vertices[i1]);
            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }
    }
    break;
    default:
        muliAssert(false);
        break;
    }
}

void DynamicRenderer::FlushPoints()
{
    shader->Use();

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * pointCount, points.data());

        glBindBuffer(GL_ARRAY_BUFFER, cVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4) * pointCount, pointColors.data());

        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pointCount));
    }
    glBindVertexArray(static_cast<GLsizei>(0));

    pointCount = 0;
}

void DynamicRenderer::FlushLines()
{
    shader->Use();

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * lineCount, lines.data());

        glBindBuffer(GL_ARRAY_BUFFER, cVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec4) * lineCount, lineColors.data());

        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineCount));
    }
    glBindVertexArray(static_cast<GLsizei>(0));

    lineCount = 0;
}

} // namespace muli