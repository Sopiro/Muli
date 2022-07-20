#include "spe/rendering/dynamic_renderer.h"

using namespace spe;

DynamicRenderer::DynamicRenderer()
{
    shader = DynamicShader::Create();

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);
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

void DynamicRenderer::Draw(const std::vector<glm::vec2>& vertices, GLenum drawMode, glm::vec3 color)
{
    shader->Use();
    shader->SetColor(std::move(color));
    shader->SetModelMatrix(glm::mat4{ 1.0f });

    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 2, vertices.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(drawMode, 0, static_cast<GLsizei>(vertices.size()));
    }
    glBindVertexArray(static_cast<GLsizei>(0));
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