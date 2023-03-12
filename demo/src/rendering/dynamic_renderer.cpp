#include "dynamic_renderer.h"

namespace muli
{

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
}

DynamicRenderer::~DynamicRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &pVBO);
    glDeleteBuffers(1, &cVBO);
}

void DynamicRenderer::DrawShape(const Shape* shape, const Transform& tf)
{
    Shape::Type type = shape->GetType();
    muliNotUsed(tf);

    switch (type)
    {
    case Shape::Type::circle:
        break;
    case Shape::Type::capsule:
        break;
    case Shape::Type::polygon:
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