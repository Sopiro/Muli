#pragma once

#include "dynamic_shader.h"
#include "renderer.h"

namespace muli
{

// Dynamic batch renderer
class DynamicRenderer final : public Renderer
{
public:
    DynamicRenderer();
    virtual ~DynamicRenderer();

    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetViewMatrix(const Mat4& viewMatrix);

    virtual void Render() const override{};
    void Draw(const std::vector<Vec2>& vertices, GLenum drawMode = GL_LINES, Vec3 color = zero_vec3);

private:
    friend class DynamicShader;

    const size_t maxVertexCount = 1024; // must be a even number

    // All registered rigid bodies
    std::unique_ptr<DynamicShader> shader;

    Mat4 viewMatrix;
    Mat4 projMatrix;

    GLuint VAO;
    GLuint VBO;
};

inline void DynamicRenderer::SetProjectionMatrix(const Mat4& _projMatrix)
{
    shader->Use();
    shader->SetProjectionMatrix(_projMatrix);
}

inline void DynamicRenderer::SetViewMatrix(const Mat4& _viewMatrix)
{
    shader->Use();
    shader->SetViewMatrix(_viewMatrix);
}

} // namespace muli