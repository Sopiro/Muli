#pragma once

#include "../common.h"
#include "shader.h"

namespace spe
{
class DynamicShader final : public Shader
{
    friend class DynamicRenderer;

public:
    static std::unique_ptr<DynamicShader> DynamicShader::Create();

    void SetColor(glm::vec3 _color);
    void SetViewMatrix(glm::mat4 _viewMatrix);
    void SetProjectionMatrix(glm::mat4 _projMatrix);
    void SetModelMatrix(glm::mat4 _modelMatrix);

private:
    DynamicShader();

    // uniforms 
    glm::vec3 color{ 0.0f };
    glm::mat4 viewMatrix{ 1.0f };
    glm::mat4 projMatrix{ 1.0f };
    glm::mat4 modelMatrix{ 1.0f };
};
}