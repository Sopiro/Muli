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

    void SetColor(Vec3 _color);
    void SetViewMatrix(Mat4 _viewMatrix);
    void SetProjectionMatrix(Mat4 _projMatrix);
    void SetModelMatrix(Mat4 _modelMatrix);

private:
    DynamicShader();

    // uniforms
    Vec3 color{ 0.0f };
    Mat4 viewMatrix{ 1.0f };
    Mat4 projMatrix{ 1.0f };
    Mat4 modelMatrix{ 1.0f };
};

inline void DynamicShader::SetColor(Vec3 _color)
{
    color = std::move(_color);
    glUniform3fv(uniformMap["color"], 1, &color.x);
}

inline void DynamicShader::SetModelMatrix(Mat4 _modelMatrix)
{
    modelMatrix = std::move(_modelMatrix);
    glUniformMatrix4fv(uniformMap["model"], 1, GL_FALSE, &modelMatrix.ex.x);
}

inline void DynamicShader::SetViewMatrix(Mat4 _viewMatrix)
{
    viewMatrix = std::move(_viewMatrix);
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, &viewMatrix.ex.x);
}

inline void DynamicShader::SetProjectionMatrix(Mat4 _projMatrix)
{
    projMatrix = std::move(_projMatrix);
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, &projMatrix.ex.x);
}

} // namespace spe