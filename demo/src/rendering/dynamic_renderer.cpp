#include "dynamic_renderer.h"

namespace muli
{

DynamicRenderer::DynamicRenderer()
{
    shader = DynamicShader::Create();

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * maxVertexCount, nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2), 0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(0);
}

DynamicRenderer::~DynamicRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

// Batch rendering: batch size = maxVertexCount
void DynamicRenderer::Draw(const std::vector<Vec2>& vertices, GLenum drawMode, Vec3 color)
{
    bool exceed = vertices.size() > maxVertexCount;

    shader->Use();
    shader->SetColor(color);
    shader->SetModelMatrix(Mat4{ 1.0f });

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        if (exceed)
        {
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * maxVertexCount, vertices.data());
            glDrawArrays(drawMode, 0, static_cast<GLsizei>(maxVertexCount));
        }
        else
        {
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Vec2) * vertices.size(), vertices.data());
            glDrawArrays(drawMode, 0, static_cast<GLsizei>(vertices.size()));
        }
    }
    glBindVertexArray(static_cast<GLsizei>(0));

    if (exceed)
    {
        Draw(std::vector<Vec2>(vertices.begin() + maxVertexCount, vertices.end()), drawMode, color);
    }
}

} // namespace muli