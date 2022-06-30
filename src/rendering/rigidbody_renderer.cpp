#include "rigidbody_renderer.h"
#include "../physics/util.h"
#include "../input.h"
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

        shader->SetColor({ 1, 1, 1 });
        mesh->Draw(GL_TRIANGLES);

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

void RigidBodyRenderer::SetViewMatrix(glm::mat4 _viewMatrix)
{
    shader->SetViewMatrix(std::move(_viewMatrix));
}

void RigidBodyRenderer::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    shader->SetProjectionMatrix(std::move(_projMatrix));
}

// Viewport space -> NDC -> world spcae
glm::vec2 RigidBodyRenderer::Pick()
{
    // Viewport space
    glm::vec2 pos = Input::GetMousePosition();
    glm::vec2 windowSize = Window::Get().GetWindowSize();

    pos.y = windowSize.y - pos.y - 1;
    pos.x /= windowSize.x;
    pos.y /= windowSize.y;
    pos -= 0.5f;
    pos *= 2.0f;
    // NDC (-1 ~ 1)

    glm::mat4 invVP = glm::inverse(shader->projMatrix * shader->viewMatrix);

    // World space
    glm::vec4 invPos = invVP * glm::vec4{ pos, 0, 1 };

    return { invPos.x, invPos.y };
}
