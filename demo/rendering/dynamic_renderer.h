#pragma once

#include "renderer.h"
#include "dynamic_shader.h"

namespace spe
{
// Dynamic batch renderer
class DynamicRenderer final : public Renderer
{
    friend class DynamicShader;

public:
    DynamicRenderer();
    virtual ~DynamicRenderer();

    void SetProjectionMatrix(glm::mat4 _projMatrix);
    void SetViewMatrix(glm::mat4 _viewMatrix);

    virtual void Render() override;
    void DynamicRenderer::Draw(const std::vector<glm::vec2>& vertices, GLenum drawMode = GL_LINES, glm::vec3 color = { 0.0f, 0.0f, 0.0f });

private:
    const uint32_t maxVertexCount = 1024; // must be a even number

    // All registered rigid bodies
    std::unique_ptr<DynamicShader> shader{};

    glm::mat4 viewMatrix;
    glm::mat4 projMatrix;

    uint32_t VAO;
    uint32_t VBO;
};
}