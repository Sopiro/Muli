#include "rigidbody_renderer.h"
#include "../physics/util.h"
#include "../window.h"

using namespace spe;

RigidBodyRenderer::RigidBodyRenderer()
{
    shader = RigidBodyShader::Create();
    shader->Use();
}

void RigidBodyRenderer::Render()
{
    shader->Use();

    for (const auto& [body, mesh] : bodiesAndMeshes)
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(body->position, 0.0f));
        glm::mat4 r = glm::rotate(glm::mat4{ 1.0f }, body->rotation, glm::vec3(0.0f, 0.0f, 1.0f));

        shader->SetModelMatrix(t * r);

        if ((!drawOutlineOnly && !body->IsSleeping()) || body->GetType() == Static)
        {
            shader->SetColor({ 1, 1, 1 });
            mesh->Draw(GL_TRIANGLES);
        }

        glLineWidth(1.5f);
        shader->SetColor({ 0, 0, 0 });
        mesh->Draw(GL_LINE_LOOP);
    }
}

void RigidBodyRenderer::Register(RigidBody* body)
{
    bodiesAndMeshes.push_back(std::pair(body, generate_mesh_from_rigidbody(*body)));
}

void RigidBodyRenderer::Register(const std::vector<RigidBody*>& bodies)
{
    for (auto b : bodies)
    {
        Register(b);
    }
}

void RigidBodyRenderer::Unregister(RigidBody* body)
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

void RigidBodyRenderer::Unregister(const std::vector<RigidBody*>& bodies)
{
    for (size_t i = 0; i < bodies.size(); i++)
    {
        Unregister(bodies[i]);
    }
}

void RigidBodyRenderer::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    shader->Use();
    shader->SetProjectionMatrix(std::move(_projMatrix));
}

void RigidBodyRenderer::SetViewMatrix(glm::mat4 _viewMatrix)
{
    shader->Use();
    shader->SetViewMatrix(std::move(_viewMatrix));
}

// Viewport space -> NDC -> world spcae
glm::vec2 RigidBodyRenderer::Pick(const glm::vec2& screenPos)
{
    // Viewport space
    glm::vec2 worldPos = screenPos;
    glm::vec2 windowSize = Window::Get().GetWindowSize();

    worldPos.y = windowSize.y - worldPos.y - 1;
    worldPos.x /= windowSize.x;
    worldPos.y /= windowSize.y;
    worldPos -= 0.5f;
    worldPos *= 2.0f;
    // NDC (-1 ~ 1)

    glm::mat4 invVP = glm::inverse(shader->projMatrix * shader->viewMatrix);

    // World space
    glm::vec4 invPos = invVP * glm::vec4{ worldPos, 0, 1 };

    return { invPos.x, invPos.y };
}

void RigidBodyRenderer::SetDrawOutlined(bool _drawOutlineOnly)
{
    drawOutlineOnly = std::move(_drawOutlineOnly);
}

void RigidBodyRenderer::Clear()
{
    bodiesAndMeshes.clear();
}
