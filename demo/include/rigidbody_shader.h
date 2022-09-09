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

    void SetColor(Vec3 _color);
    void SetViewMatrix(Mat4 _viewMatrix);
    void SetProjectionMatrix(Mat4 _projMatrix);
    void SetModelMatrix(Mat4 _modelMatrix);

private:
    RigidBodyShader();

    // uniforms
    Vec3 color{ 0.0f };
    Mat4 viewMatrix{ 1.0f };
    Mat4 projMatrix{ 1.0f };
    Mat4 modelMatrix{ 1.0f };
};

inline void RigidBodyShader::SetColor(Vec3 _color)
{
    color = std::move(_color);
    glUniform3fv(uniformMap["color"], 1, &color.x);
}

inline void RigidBodyShader::SetModelMatrix(Mat4 _modelMatrix)
{
    modelMatrix = std::move(_modelMatrix);
    glUniformMatrix4fv(uniformMap["model"], 1, GL_FALSE, &modelMatrix[0][0]);
}

inline void RigidBodyShader::SetViewMatrix(Mat4 _viewMatrix)
{
    viewMatrix = std::move(_viewMatrix);
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, &viewMatrix[0][0]);
}

inline void RigidBodyShader::SetProjectionMatrix(Mat4 _projMatrix)
{
    projMatrix = std::move(_projMatrix);
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, &projMatrix[0][0]);
}

} // namespace spe