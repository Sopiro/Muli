#pragma once

#include "common.h"
#include "mesh.h"
#include "renderer.h"
#include "rigidbody_shader.h"
#include "util.h"

namespace spe
{

class RigidBodyRenderer final : public Renderer
{
    friend class RigidBodyShader;

public:
    RigidBodyRenderer();

    virtual void Render() override;

    void SetProjectionMatrix(const Mat4& _projMatrix);
    void SetViewMatrix(const Mat4& _viewMatrix);

    void Register(RigidBody* body);
    void Register(const std::vector<RigidBody*>& bodies);
    void Unregister(RigidBody* body);
    void Unregister(const std::vector<RigidBody*>& bodies);

    Vec2 Pick(const Vec2& screenPos);

    void SetDrawOutlined(bool drawOutlineOnly);
    void Reset();

private:
    // All registered rigid bodies
    std::unique_ptr<RigidBodyShader> shader{};
    std::vector<std::pair<RigidBody*, std::unique_ptr<Mesh>>> bodiesAndMeshes{};

    Mat4 viewMatrix;
    Mat4 projMatrix;

    bool drawOutlineOnly;
};

inline void RigidBodyRenderer::Register(RigidBody* body)
{
    bodiesAndMeshes.push_back(std::pair(body, GenerateMesh(*body)));
}

inline void RigidBodyRenderer::Register(const std::vector<RigidBody*>& bodies)
{
    for (auto b : bodies)
    {
        Register(b);
    }
}

inline void RigidBodyRenderer::Unregister(RigidBody* body)
{
    size_t idx = bodiesAndMeshes.size();

    for (size_t i = 0; i < bodiesAndMeshes.size(); i++)
    {
        if (bodiesAndMeshes[i].first == body)
        {
            idx = i;
            break;
        }
    }

    if (idx < bodiesAndMeshes.size())
    {
        bodiesAndMeshes.erase(bodiesAndMeshes.begin() + idx);
    }
}

inline void RigidBodyRenderer::Unregister(const std::vector<RigidBody*>& bodies)
{
    for (uint32 i = 0; i < bodies.size(); i++)
    {
        Unregister(bodies[i]);
    }
}

inline void RigidBodyRenderer::SetProjectionMatrix(const Mat4& _projMatrix)
{
    shader->Use();
    shader->SetProjectionMatrix(_projMatrix);
}

inline void RigidBodyRenderer::SetViewMatrix(const Mat4& _viewMatrix)
{
    shader->Use();
    shader->SetViewMatrix(_viewMatrix);
}

inline void RigidBodyRenderer::SetDrawOutlined(bool _drawOutlineOnly)
{
    drawOutlineOnly = _drawOutlineOnly;
}

inline void RigidBodyRenderer::Reset()
{
    bodiesAndMeshes.clear();
}

} // namespace spe