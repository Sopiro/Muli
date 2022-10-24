#include "rigidbody_renderer.h"
#include "muli/util.h"
#include "options.h"
#include "window.h"

namespace muli
{

RigidBodyRenderer::RigidBodyRenderer()
{
    shader = RigidBodyShader::Create();
    shader->Use();
}

void RigidBodyRenderer::Render()
{
    shader->Use();

    for (uint32 i = 0; i < bodiesAndMeshes.size(); i++)
    {
        RigidBody* body = bodiesAndMeshes[i].first;
        RigidBody::Type type = body->GetType();

        std::unique_ptr<Mesh>& mesh = bodiesAndMeshes[i].second;

        Mat4 t = Mat4{ 1.0f }.Translate(body->GetPosition().x, body->GetPosition().y, 0.0f);
        Mat4 r = Mat4{ 1.0f }.Rotate(0.0f, 0.0f, body->GetAngle());

        shader->SetModelMatrix(t * r);

        if ((!drawOutlineOnly && !body->IsSleeping()) || type == RigidBody::Type::Static)
        {
            Vec3 color{ 1.0f };
            const CollisionFilter& cf = body->GetCollisionFilter();

            switch (type)
            {
            case RigidBody::Type::Dynamic:
                float h, s, l;

                if (cf.group == 1 && cf.mask != 0xffffffff)
                {
                    h = (((cf.filter - 2) * 17) % 360) / 360.0f;
                    s = 100.0f / 100.0f;
                    l = 75.0f / 100.0f;
                }
                else
                {
                    int32 id = body->GetIslandID();

                    int32 hStride = 17;
                    int32 sStride = 5;
                    int32 lStride = 3;
                    int32 period = static_cast<int32>(Trunc(360.0f / hStride));
                    int32 cycle = static_cast<int32>(Trunc((float)id / period));

                    h = (((id - 1) * hStride) % 360) / 360.0f;
                    s = (100 - (cycle * sStride) % 21) / 100.0f;
                    l = (75 - (cycle * lStride) % 17) / 100.0f;
                }

                color = hsl2rgb(h, s, l);
                break;

            case RigidBody::Type::Kinematic:
                color.Set(0.83f, 0.82f, 0.84f);
                break;

            case RigidBody::Type::Static:
                if (cf.group == 1 && cf.mask != 0xffffffff)
                {
                    float h = (((cf.filter - 2) * 17) % 360) / 360.0f;
                    float s = 100.0f / 100.0f;
                    float l = 75.0f / 100.0f;

                    color = hsl2rgb(h, s, l);
                }
                break;
            }

            shader->SetColor({ color.x, color.y, color.z });
            mesh->Draw(GL_TRIANGLES);
        }

        if (!(body->userFlag & UserFlag::REMOVE_OUTLINE))
        {
            glLineWidth(1.0f);
            shader->SetColor({ 0, 0, 0 });
            mesh->Draw(GL_LINE_LOOP);
        }
    }
}

// Viewport space -> NDC -> world spcae
Vec2 RigidBodyRenderer::Pick(const Vec2& screenPos) const
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

} // namespace muli