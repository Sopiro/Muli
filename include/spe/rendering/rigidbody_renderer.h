#pragma once

#include "../common.h"
#include "renderer.h"
#include "../physics/rigidbody.h"
#include "rigidbody_shader.h"

namespace spe
{
class RigidBodyRenderer final : public Renderer
{
    friend class RigidBodyShader;

public:
    RigidBodyRenderer();

    void SetProjectionMatrix(glm::mat4 _projMatrix);
    void SetViewMatrix(glm::mat4 _viewMatrix);

    virtual void Render() override;

    void Register(RigidBody* body);
    void Register(const std::vector<RigidBody*>& bodies);
    void Unregister(RigidBody* body);
    void Unregister(const std::vector<RigidBody*>& bodies);

    glm::vec2 Pick(const glm::vec2& screenPos);

    void SetDrawOutlined(bool drawOutlineOnly);

    void Reset();

private:
    // All registered rigid bodies
    std::unique_ptr<RigidBodyShader> shader{};
    std::vector<std::pair<RigidBody*, std::unique_ptr<Mesh>>> bodiesAndMeshes{};

    glm::mat4 viewMatrix;
    glm::mat4 projMatrix;

    bool drawOutlineOnly;
};
}