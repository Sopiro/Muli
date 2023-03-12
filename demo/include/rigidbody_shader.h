#pragma once

#include "common.h"
#include "shader.h"

namespace muli
{

class RigidBodyShader final : public Shader
{
public:
    static std::unique_ptr<RigidBodyShader> Create();

    void SetColor(const Vec4& rgba);
    void SetColor(const Vec3& rgb, float a = 1.0f);
    void SetViewMatrix(const Mat4& viewMatrix);
    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetModelMatrix(const Mat4& modelMatrix);

private:
    friend class RigidBodyRenderer;

    RigidBodyShader();

    // uniforms
    Vec4 color{ zero_vec3, 1.0f };
    Mat4 viewMatrix{ identity };
    Mat4 projMatrix{ identity };
    Mat4 modelMatrix{ identity };
};

inline std::unique_ptr<RigidBodyShader> RigidBodyShader::Create()
{
    return std::unique_ptr<RigidBodyShader>(new RigidBodyShader);
}

inline void RigidBodyShader::SetColor(const Vec4& rgba)
{
    color = rgba;
    glUniform4fv(uniformMap["color"], 1, &color.x);
}

inline void RigidBodyShader::SetColor(const Vec3& rgb, float a)
{
    color.Set(rgb.x, rgb.y, rgb.z, a);
    glUniform4fv(uniformMap["color"], 1, &color.x);
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