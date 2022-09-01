#include "rigidbody_renderer.h"
#include "spe/util.h"
#include "window.h"

namespace spe
{

RigidBodyRenderer::RigidBodyRenderer()
{
    shader = RigidBodyShader::Create();
    shader->Use();
}

void RigidBodyRenderer::Render()
{
    shader->Use();

    for (uint32_t i = 0; i < bodiesAndMeshes.size(); i++)
    {
        RigidBody* body = bodiesAndMeshes[i].first;
        std::unique_ptr<Mesh>& mesh = bodiesAndMeshes[i].second;

        glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(body->position, 0.0f));
        glm::mat4 r = glm::rotate(glm::mat4{ 1.0f }, body->rotation, glm::vec3(0.0f, 0.0f, 1.0f));

        shader->SetModelMatrix(t * r);

        if ((!drawOutlineOnly && !body->IsSleeping()) || body->GetType() == RigidBody::Type::Static)
        {
            glm::vec3 color{ 1.0f };

            if (body->GetType() == RigidBody::Type::Dynamic)
            {
                int32_t id = body->GetIslandID();

                int32_t hStride = 17;
                int32_t sStride = 5;
                int32_t lStride = 3;
                int32_t period = static_cast<int32_t>(glm::trunc(360 / hStride));
                int32_t cycle = static_cast<int32_t>(glm::trunc(id / period));

                float h = (((id - 1) * hStride) % 360) / 360.0f;
                float s = (100 - (cycle * sStride) % 21) / 100.0f;
                float l = (75 - (cycle * lStride) % 17) / 100.0f;

                color = hsl2rgb(h, s, l);
            }

            shader->SetColor({ color.r, color.g, color.b });
            mesh->Draw(GL_TRIANGLES);
        }

        glLineWidth(1.5f);
        shader->SetColor({ 0, 0, 0 });
        mesh->Draw(GL_LINE_LOOP);
    }
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

} // namespace spe