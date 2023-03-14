#include "renderer.h"
#include "util.h"

namespace muli
{

Renderer::Renderer()
    : pointCount{ 0 }
    , lineCount{ 0 }
    , triangleCount{ 0 }
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

        constexpr float stride = 360.0f / color_count;
        for (int32 i = 0; i < color_count; ++i)
        {
            Vec3 rgb = hsl2rgb(i * stride / 360.0f, 1.0f, 0.75f);

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
}

void Renderer::DrawShapeOutlined(const Shape* shape, const Transform& tf, bool rounded)
{
    switch (shape->GetType())
    {
    case Shape::Type::circle:
    {
        const Circle* c = static_cast<const Circle*>(shape);

        float radius = c->GetRadius();
        Vec2 localCenter = c->GetCenter();
        Vec2 center = Mul(tf, localCenter);

        int32 i0 = circle_count - 1;
        int32 i1 = 0;
        Vec2 v0 = Mul(tf, localCenter + unit_circle[i0] * radius);
        Vec2 v1 = Mul(tf, localCenter + unit_circle[i1] * radius);

        DrawLine(center, v1);
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

        Vec2 gva = Mul(tf, va);
        Vec2 gvb = Mul(tf, vb);

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
        if (rounded == false)
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
        else
        {
            const Polygon* p = static_cast<const Polygon*>(shape);
            float radius = p->GetRadius();

            const Vec2* vertices = p->GetVertices();
            const Vec2* normals = p->GetNormals();
            int32 vertexCount = p->GetVertexCount();

            int32 i0 = vertexCount - 1;
            Vec2 v0 = vertices[i0];
            Vec2 n0 = normals[i0];
            Vec2 vn0 = v0 + n0 * radius;
            Vec2 gv0 = Mul(tf, vn0);

            for (int32 i1 = 0; i1 < vertexCount; ++i1)
            {
                Vec2 v1 = vertices[i1];
                Vec2 n1 = normals[i1];
                Vec2 vn1 = v1 + n1 * radius;

                float theta = AngleBetween(n0, n1);
                float cCount = circle_count * theta / (2.0f * pi);

                Vec2 gv1;
                for (int32 j = 0; j < cCount; ++j)
                {
                    float per = j / cCount;
                    Vec2 cv = v1 + Slerp(n0, n1, per) * radius;

                    gv1 = Mul(tf, cv);
                    DrawLine(gv0, gv1);
                    gv0 = gv1;
                }

                gv1 = Mul(tf, vn1);

                DrawLine(gv0, gv1);

                i0 = i1;
                v0 = v1;
                n0 = n1;
                vn0 = vn1;
                gv0 = gv1;
            }
        }
    }
    break;
    default:
        muliAssert(false);
        break;
    }
}

void Renderer::DrawShapeSolid(const Shape* shape, const Transform& tf, int32 colorIndex, bool rounded)
{
    const Vec4& color = colorIndex < 0 ? color_white : colors[colorIndex % color_count];

    switch (shape->GetType())
    {
    case Shape::Type::circle:
    {
        const Circle* c = static_cast<const Circle*>(shape);
        Vec2 localCenter = c->GetCenter();
        Vec2 center = Mul(tf, localCenter);
        float radius = c->GetRadius();

        int32 i0 = circle_count - 1;
        int32 i1 = 0;
        Vec2 v0 = Mul(tf, localCenter + unit_circle[i0] * radius);
        Vec2 v1 = Mul(tf, localCenter + unit_circle[i1] * radius);

        DrawTriangle(center, v0, v1, color);
        DrawLine(center, v1);
        DrawLine(v0, v1);

        i0 = i1;
        v0 = v1;

        for (i1 = 1; i1 < circle_count; ++i1)
        {
            v1 = Mul(tf, localCenter + unit_circle[i1] * radius);
            DrawTriangle(center, v0, v1, color);
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

        Vec2 gva = Mul(tf, va);
        Vec2 gvb = Mul(tf, vb);

        Vec2 dir = vb - va;
        float angleOffset = AngleBetween(Vec2{ 1.0f, 0.0 }, dir);

        Rotation rot{ angleOffset };

        float radius = c->GetRadius();

        int32 i0 = 0;
        Vec2 vf = Mul(tf, vb + Mul(rot, unit_circle[i0] * radius));
        Vec2 v0 = vf;
        Vec2 v1;

        for (int32 i1 = 1; i0 < quater_count; ++i1)
        {
            v1 = Mul(tf, vb + Mul(rot, unit_circle[i1] * radius));

            DrawTriangle(gvb, v0, v1, color);
            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }

        DrawTriangle(gvb, v0, gva, color);

        for (int32 i1 = quater_count; i0 < quater_count * 3; ++i1)
        {
            v1 = Mul(tf, va + Mul(rot, unit_circle[i1] * radius));

            DrawTriangle(gva, v0, v1, color);
            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }

        DrawTriangle(gva, v0, gvb, color);

        for (int32 i1 = quater_count * 3; i0 < quater_count * 4 - 1; ++i1)
        {
            v1 = Mul(tf, vb + Mul(rot, unit_circle[i1] * radius));

            DrawTriangle(gvb, v0, v1, color);
            DrawLine(v0, v1);

            i0 = i1;
            v0 = v1;
        }

        DrawTriangle(gvb, v1, vf, color);
        DrawLine(v1, vf);
    }
    break;
    case Shape::Type::polygon:
    {
        if (rounded == false)
        {
            const Polygon* p = static_cast<const Polygon*>(shape);
            float radius = p->GetRadius();

            const Vec2* vertices = p->GetVertices();
            int32 vertexCount = p->GetVertexCount();

            Vec2 center = Mul(tf, p->GetCenter());

            int32 i0 = vertexCount - 1;
            Vec2 v0 = Mul(tf, vertices[i0]);
            for (int32 i1 = 0; i1 < vertexCount; ++i1)
            {
                Vec2 v1 = Mul(tf, vertices[i1]);

                DrawTriangle(center, v0, v1, color);
                DrawLine(v0, v1);

                i0 = i1;
                v0 = v1;
            }
        }
        else
        {
            const Polygon* p = static_cast<const Polygon*>(shape);
            float radius = p->GetRadius();

            const Vec2* vertices = p->GetVertices();
            const Vec2* normals = p->GetNormals();
            int32 vertexCount = p->GetVertexCount();

            Vec2 center = Mul(tf, p->GetCenter());

            int32 i0 = vertexCount - 1;
            Vec2 v0 = vertices[i0];
            Vec2 n0 = normals[i0];
            Vec2 vn0 = v0 + n0 * radius;
            Vec2 gv0 = Mul(tf, vn0);

            for (int32 i1 = 0; i1 < vertexCount; ++i1)
            {
                Vec2 v1 = vertices[i1];
                Vec2 n1 = normals[i1];
                Vec2 vn1 = v1 + n1 * radius;

                float theta = AngleBetween(n0, n1);
                float cCount = circle_count * theta / (2.0f * pi);

                Vec2 gv1;
                for (int32 j = 0; j < cCount; ++j)
                {
                    float per = j / cCount;
                    Vec2 cv = v1 + Slerp(n0, n1, per) * radius;

                    gv1 = Mul(tf, cv);
                    DrawTriangle(center, gv0, gv1, color);
                    DrawLine(gv0, gv1);
                    gv0 = gv1;
                }

                gv1 = Mul(tf, vn1);

                DrawTriangle(center, gv0, gv1, color);
                DrawLine(gv0, gv1);

                i0 = i1;
                v0 = v1;
                n0 = n1;
                vn0 = vn1;
                gv0 = gv1;
            }
        }
    }
    break;
    default:
        muliAssert(false);
        break;
    }
}

void Renderer::FlushPoints()
{
    shader->Use();

    glBindVertexArray(VAO);
    {
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