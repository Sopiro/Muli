#pragma once

#include "common.h"
#include "shader.h"

namespace muli
{

class DynamicShader final : public Shader
{
public:
    static std::unique_ptr<DynamicShader> Create();

    const Mat4& GetViewMatrix() const;
    void SetViewMatrix(const Mat4& viewMatrix);
    const Mat4& GetProjectionMatrix() const;
    void SetProjectionMatrix(const Mat4& projMatrix);

private:
    friend class Renderer;

    DynamicShader();

    Mat4 viewMatrix{ identity };
    Mat4 projMatrix{ identity };
};

inline std::unique_ptr<DynamicShader> DynamicShader::Create()
{
    return std::unique_ptr<DynamicShader>(new DynamicShader);
}

inline const Mat4& DynamicShader::GetViewMatrix() const
{
    return viewMatrix;
}

inline void DynamicShader::SetViewMatrix(const Mat4& _viewMatrix)
{
    viewMatrix = _viewMatrix;
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, &viewMatrix[0][0]);
}

inline const Mat4& DynamicShader::GetProjectionMatrix() const
{
    return projMatrix;
}

inline void DynamicShader::SetProjectionMatrix(const Mat4& _projMatrix)
{
    projMatrix = _projMatrix;
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, &projMatrix[0][0]);
}

} // namespace muli