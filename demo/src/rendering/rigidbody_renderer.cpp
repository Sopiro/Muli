#include "rigidbody_renderer.h"
#include "game.h"
#include "muli/util.h"
#include "window.h"

namespace muli
{

RigidBodyRenderer::RigidBodyRenderer(Game& _game)
    : game{ _game }
    , options{ _game.GetDebugOptions() }
{
    shader = RigidBodyShader::Create();
    shader->Use();
}

void RigidBodyRenderer::Render() const
{
    shader->Use();

    for (size_t i = 0; i < collidersAndMeshes.size(); ++i)
    {
        Collider* collider = collidersAndMeshes[i].first;
        RigidBody* body = collider->GetBody();

        RigidBody::Type type = body->GetType();

        const std::unique_ptr<Mesh>& mesh = collidersAndMeshes[i].second;

        Mat4 t{ body->GetTransform() };
        shader->SetModelMatrix(t);

        if ((options.draw_outline_only == false && body->IsSleeping() == false))
        {
            Vec3 color{ 1.0f };
            const CollisionFilter& cf = collider->GetFilter();

            switch (type)
            {
            case RigidBody::Type::dynamic_body:
                float h, s, l;

                if (cf.group == default_collision_filter.group && cf.mask != default_collision_filter.mask)
                {
                    h = (((cf.bit - 2) * 17) % 360) / 360.0f;
                    s = 100.0f / 100.0f;
                    l = 75.0f / 100.0f;
                }
                else
                {
                    if (options.colorize_island)
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
                    else
                    {
                        h = 1.0f;
                        s = 1.0f;
                        l = 0.75f;
                    }
                }

                color = hsl2rgb(h, s, l);
                break;

            case RigidBody::Type::kinematic_body:
                color.Set(0.83f, 0.82f, 0.84f);
                break;

            case RigidBody::Type::static_body:
                if (cf.group == default_collision_filter.group && cf.mask != default_collision_filter.mask)
                {
                    float h = (((cf.bit - 2) * 17) % 360) / 360.0f;
                    float s = 100.0f / 100.0f;
                    float l = 75.0f / 100.0f;

                    color = hsl2rgb(h, s, l);
                }
                break;
            }

            shader->SetColor(color);
            mesh->Draw(GL_TRIANGLES);
        }

        if (!(body->UserFlag & UserFlag::remove_outline))
        {
            glLineWidth(1.0f);
            shader->SetColor(zero_vec3);
            mesh->Draw(GL_LINE_LOOP);
        }
    }
}

void RigidBodyRenderer::Render(Collider* collider, const Transform& transform) const
{
    for (size_t i = 0; i < collidersAndMeshes.size(); ++i)
    {
        Collider* c = collidersAndMeshes[i].first;
        if (c != collider)
        {
            continue;
        }

        RigidBody* body = c->GetBody();
        Mat4 t = Mat4{ transform };

        shader->Use();
        shader->SetModelMatrix(t);
        shader->SetColor(Vec3{ 0.2f });

        const std::unique_ptr<Mesh>& mesh = collidersAndMeshes[i].second;
        mesh->Draw(GL_LINE_LOOP);

        break;
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
    Vec4 invPos = invVP * Vec4{ worldPos.x, worldPos.y, 0.0f, 1.0f };

    return Vec2{ invPos.x, invPos.y };
}

} // namespace muli