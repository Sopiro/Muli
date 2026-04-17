#pragma once

#include "shader.h"

namespace muli
{

class ShapeSdfShader final : public Shader
{
public:
    static std::unique_ptr<ShapeSdfShader> Create();

    void SetViewMatrix(const Mat4& viewMatrix);
    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetPixelWorldSize(const Vec2& pixelWorldSize);
    void SetPolygonVertexTextureUnit(int32 textureUnit);

private:
    ShapeSdfShader();
};

inline std::unique_ptr<ShapeSdfShader> ShapeSdfShader::Create()
{
    return std::unique_ptr<ShapeSdfShader>(new ShapeSdfShader);
}

inline void ShapeSdfShader::SetViewMatrix(const Mat4& viewMatrix)
{
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, &viewMatrix.ex.x);
}

inline void ShapeSdfShader::SetProjectionMatrix(const Mat4& projMatrix)
{
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, &projMatrix.ex.x);
}

inline void ShapeSdfShader::SetPixelWorldSize(const Vec2& pixelWorldSize)
{
    glUniform2fv(uniformMap["pixelWorldSize"], 1, &pixelWorldSize.x);
}

inline void ShapeSdfShader::SetPolygonVertexTextureUnit(int32 textureUnit)
{
    glUniform1i(uniformMap["polygonVerticesTex"], textureUnit);
}

} // namespace muli
