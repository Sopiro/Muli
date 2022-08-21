#pragma once

#include "common.h"
#include "shader.h"

namespace spe
{
class RigidBodyShader final : public Shader
{
    friend class RigidBodyRenderer;

public:
    static std::unique_ptr<RigidBodyShader> RigidBodyShader::Create();

    inline void SetColor(glm::vec3 _color);
    inline void SetViewMatrix(glm::mat4 _viewMatrix);
    inline void SetProjectionMatrix(glm::mat4 _projMatrix);
    inline void SetModelMatrix(glm::mat4 _modelMatrix);

private:
    RigidBodyShader();

    // uniforms 
    glm::vec3 color{ 0.0f };
    glm::mat4 viewMatrix{ 1.0f };
    glm::mat4 projMatrix{ 1.0f };
    glm::mat4 modelMatrix{ 1.0f };
};

inline void RigidBodyShader::SetColor(glm::vec3 _color)
{
    color = std::move(_color);
    glUniform3fv(uniformMap["color"], 1, &color.r);
}

inline void RigidBodyShader::SetModelMatrix(glm::mat4 _modelMatrix)
{
    modelMatrix = std::move(_modelMatrix);
    glUniformMatrix4fv(uniformMap["model"], 1, GL_FALSE, glm::value_ptr(modelMatrix));
}

inline void RigidBodyShader::SetViewMatrix(glm::mat4 _viewMatrix)
{
    viewMatrix = std::move(_viewMatrix);
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, glm::value_ptr(viewMatrix));
}

inline void RigidBodyShader::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    projMatrix = std::move(_projMatrix);
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, glm::value_ptr(projMatrix));
}

}