#include "dynamic_renderer.h"

namespace spe
{

DynamicRenderer::DynamicRenderer()
{
    shader = DynamicShader::Create();

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * maxVertexCount, nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), 0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(0);
}

DynamicRenderer::~DynamicRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void DynamicRenderer::Render() {}

// Batch rendering: batch size = maxVertexCount
void DynamicRenderer::Draw(const std::vector<glm::vec2>& vertices, GLenum drawMode, glm::vec3 color)
{
    bool exceed = vertices.size() > maxVertexCount;

    shader->Use();
    shader->SetColor(color);
    shader->SetModelMatrix(glm::mat4{ 1.0f });

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        if (exceed)
        {
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec2) * maxVertexCount, vertices.data());
            glDrawArrays(drawMode, 0, static_cast<GLsizei>(maxVertexCount));
        }
        else
        {
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec2) * vertices.size(), vertices.data());
            glDrawArrays(drawMode, 0, static_cast<GLsizei>(vertices.size()));
        }
    }
    glBindVertexArray(static_cast<GLsizei>(0));

    if (exceed)
    {
        Draw(std::vector<glm::vec2>(vertices.begin() + maxVertexCount, vertices.end()), drawMode, color);
    }
}

void DynamicRenderer::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    shader->Use();
    shader->SetProjectionMatrix(std::move(_projMatrix));
}

void DynamicRenderer::SetViewMatrix(glm::mat4 _viewMatrix)
{
    shader->Use();
    shader->SetViewMatrix(std::move(_viewMatrix));
}

}