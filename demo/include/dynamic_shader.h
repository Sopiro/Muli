#pragma once

#include "common.h"
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

inline void DynamicShader::SetColor(glm::vec3 _color)
{
    color = std::move(_color);
    glUniform3fv(uniformMap["color"], 1, &color.r);
}

inline void DynamicShader::SetModelMatrix(glm::mat4 _modelMatrix)
{
    modelMatrix = std::move(_modelMatrix);
    glUniformMatrix4fv(uniformMap["model"], 1, GL_FALSE, glm::value_ptr(modelMatrix));
}

inline void DynamicShader::SetViewMatrix(glm::mat4 _viewMatrix)
{
    viewMatrix = std::move(_viewMatrix);
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, glm::value_ptr(viewMatrix));
}

inline void DynamicShader::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    projMatrix = std::move(_projMatrix);
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, glm::value_ptr(projMatrix));
}

}