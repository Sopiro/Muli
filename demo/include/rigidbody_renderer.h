#pragma once

#include "common.h"
#include "mesh.h"
#include "options.h"
#include "renderer.h"
#include "rigidbody_shader.h"
#include "util.h"

namespace muli
{

class Game;

class RigidBodyRenderer final : public Renderer
{
public:
    RigidBodyRenderer(Game& game);

    virtual void Render() const override;
    void Render(Collider* c, const Transform& t) const;

    void SetProjectionMatrix(const Mat4& projMatrix);
    void SetViewMatrix(const Mat4& viewMatrix);

    void Register(Collider* body);
    void Register(const std::vector<Collider*>& bodies);
    void Unregister(Collider* body);
    void Unregister(const std::vector<Collider*>& bodies);

    Vec2 Pick(const Vec2& screenPos) const;

    void Reset();

private:
    friend class RigidBodyShader;

    Game& game;
    DebugOptions& options;

    // All registered colliders
    std::unique_ptr<RigidBodyShader> shader{};
    std::vector<std::pair<Collider*, std::unique_ptr<Mesh>>> collidersAndMeshes{};

    Mat4 viewMatrix;
    Mat4 projMatrix;
};

inline void RigidBodyRenderer::Register(Collider* collider)
{
    collidersAndMeshes.push_back(std::make_pair(collider, GenerateMesh(collider)));
}

inline void RigidBodyRenderer::Register(const std::vector<Collider*>& colliders)
{
    for (auto c : colliders)
    {
        Register(c);
    }
}

inline void RigidBodyRenderer::Unregister(Collider* collider)
{
    size_t idx = collidersAndMeshes.size();

    for (size_t i = 0; i < collidersAndMeshes.size(); ++i)
    {
        if (collidersAndMeshes[i].first == collider)
        {
            idx = i;
            break;
        }
    }

    if (idx < collidersAndMeshes.size())
    {
        collidersAndMeshes.erase(collidersAndMeshes.begin() + idx);
    }
}

inline void RigidBodyRenderer::Unregister(const std::vector<Collider*>& colliders)
{
    for (size_t i = 0; i < colliders.size(); ++i)
    {
        Unregister(colliders[i]);
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

inline void RigidBodyRenderer::Reset()
{
    collidersAndMeshes.clear();
}

} // namespace muli