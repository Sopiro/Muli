#pragma once

#include "common.h"
#include "shader.h"

namespace muli
{

class DynamicShader final : public Shader
{
public:
    static std::unique_ptr<DynamicShader> Create();

    void SetViewMatrix(const Mat4& viewMatrix);
    void SetProjectionMatrix(const Mat4& projMatrix);

private:
    friend class DynamicRenderer;

    DynamicShader();

    Mat4 viewMatrix{ identity };
    Mat4 projMatrix{ identity };
};

inline void DynamicShader::SetViewMatrix(const Mat4& _viewMatrix)
{
    viewMatrix = _viewMatrix;
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, &viewMatrix[0][0]);
}

inline void DynamicShader::SetProjectionMatrix(const Mat4& _projMatrix)
{
    projMatrix = _projMatrix;
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, &projMatrix[0][0]);
}

} // namespace muli