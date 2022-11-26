#pragma once

#include "common.h"
#include "shader.h"

namespace muli
{

class RigidBodyShader final : public Shader
{
public:
    static std::unique_ptr<RigidBodyShader> Create();

    void SetColor(const Vec3& color);
    void SetViewMatrix(const Mat4& viewMatrix);
    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetModelMatrix(const Mat4& modelMatrix);

private:
    friend class RigidBodyRenderer;

    RigidBodyShader();

    // uniforms
    Vec3 color{ 0.0f };
    Mat4 viewMatrix{ 1.0f };
    Mat4 projMatrix{ 1.0f };
    Mat4 modelMatrix{ 1.0f };
};

inline void RigidBodyShader::SetColor(const Vec3& _color)
{
    color = _color;
    glUniform3fv(uniformMap["color"], 1, &color.x);
}

inline void RigidBodyShader::SetModelMatrix(const Mat4& _modelMatrix)
{
    modelMatrix = _modelMatrix;
    glUniformMatrix4fv(uniformMap["model"], 1, GL_FALSE, &modelMatrix[0][0]);
}

inline void RigidBodyShader::SetViewMatrix(const Mat4& _viewMatrix)
{
    viewMatrix = _viewMatrix;
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, &viewMatrix[0][0]);
}

inline void RigidBodyShader::SetProjectionMatrix(const Mat4& _projMatrix)
{
    projMatrix = _projMatrix;
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, &projMatrix[0][0]);
}

} // namespace muli