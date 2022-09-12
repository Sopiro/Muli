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

        Mat4 t = Mat4{ 1.0f }.Translate(body->GetPosition().x, body->GetPosition().y, 0.0f);
        Mat4 r = Mat4{ 1.0f }.Rotate(0.0f, 0.0f, body->GetAngle());

        shader->SetModelMatrix(t * r);

        if ((!drawOutlineOnly && !body->IsSleeping()) || body->GetType() == RigidBody::Type::Static)
        {
            Vec3 color{ 1.0f };

            if (body->GetType() == RigidBody::Type::Dynamic)
            {
                int32_t id = body->GetIslandID();

                int32_t hStride = 17;
                int32_t sStride = 5;
                int32_t lStride = 3;
                int32_t period = static_cast<int32_t>(Trunc(360.0f / hStride));
                int32_t cycle = static_cast<int32_t>(Trunc((float)id / period));

                float h = (((id - 1) * hStride) % 360) / 360.0f;
                float s = (100 - (cycle * sStride) % 21) / 100.0f;
                float l = (75 - (cycle * lStride) % 17) / 100.0f;

                color = hsl2rgb(h, s, l);
            }

            shader->SetColor({ color.x, color.y, color.z });
            mesh->Draw(GL_TRIANGLES);
        }

        glLineWidth(1.5f);
        shader->SetColor({ 0, 0, 0 });
        mesh->Draw(GL_LINE_LOOP);
    }
}

// Viewport space -> NDC -> world spcae
Vec2 RigidBodyRenderer::Pick(const Vec2& screenPos)
{
    // Viewport space
    Vec2 worldPos = screenPos;
    Vec2 windowSize = Window::Get().GetWindowSize();

    worldPos.y = windowSize.y - worldPos.y - 1;
    worldPos.x /= windowSize.x;
    worldPos.y /= windowSize.y;
    worldPos -= 0.5f;
    worldPos *= 2.0f;
    // NDC (-1 ~ 1)

    Mat4 invVP = (shader->projMatrix * shader->viewMatrix).GetInverse();

    // World space
    Vec4 invPos = invVP * Vec4{ worldPos.x, worldPos.y, 0, 1 };

    return { invPos.x, invPos.y };
}

} // namespace spe