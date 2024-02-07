#include "demo.h"
#include "game.h"
#include "window.h"

namespace muli
{

class ShapeCastWorld : public Demo
{
public:
    ShapeCastOutput output;

    Vec2 translation{ -5.0f, 0.0f };
    bool closest = true;

    ShapeCastWorld(Game& game)
        : Demo(game)
    {
        options.show_contact_normal = true;
        options.show_contact_point = true;
        settings.apply_gravity = false;
        settings.sleeping = false;

        RigidBody* b = world->CreateBox(0.5f);
        b->SetPosition(0.0f, 5.5f);
        b->SetRotation(pi / 3);

        b = world->CreateRegularPolygon(0.3, 3, 0.0f, RigidBody::dynamic_body, 0.1f);
        b->SetPosition(0.0f, 4.0f);
        b->SetRotation(pi / 2);
        UserFlag::SetFlag(b, UserFlag::render_polygon_radius, true);

        b = world->CreateCapsule(0.5, 0.25f);
        b->SetPosition(0.0f, 2.5f);
        b->SetRotation(pi / 4);

        b = world->CreateCircle(0.3f);
        b->SetPosition(0.0f, 1.0f);

        b = world->CreateCapsule(0.7f, 0.3f);
        // RigidBody* b = world->CreateCircle(0.5f);
        // RigidBody* b = world->CreateBox(0.5f);
        b->SetPosition(3.0f, 3.0f);
        b->SetFixedRotation(true);
        b->SetContinuous(true);
    }

    void Render() override
    {
        if (world->GetBodyCount() > 1)
        {
            RigidBody* body = world->GetBodyListTail();
            Collider* collider = body->GetColliderList();
            const Shape* shape = collider->GetShape();
            Transform tf0 = body->GetTransform();

            Renderer& renderer = game.GetRenderer();

            Renderer::DrawMode drawMode;
            drawMode.fill = false;
            drawMode.outline = true;

            if (!closest)
            {
                struct TempAnyCallback : public ShapeCastAnyCallback
                {
                    Renderer* r;

                    Transform tf0;
                    Vec2 translation;
                    const Shape* shape;

                    bool hit = false;

                    float OnHitAny(Collider* collider, const Vec2& point, const Vec2& normal, float t)
                    {
                        Renderer::DrawMode drawMode;
                        drawMode.fill = false;
                        drawMode.outline = true;

                        Transform tf1 = tf0;
                        tf1.position += translation * t;

                        r->DrawShape(shape, tf1, drawMode);
                        r->DrawLine(tf0.position, tf1.position);
                        r->DrawPoint(point);
                        r->DrawLine(point, point + normal * 0.1f);

                        hit = true;

                        return 1.0f;
                    }

                } callback;

                callback.r = &renderer;
                callback.translation = translation;
                callback.tf0 = tf0;
                callback.shape = shape;
                world->ShapeCastAny(shape, tf0, translation, &callback);

                if (callback.hit == false)
                {
                    Transform tf1 = tf0;
                    tf1.position += translation;

                    renderer.DrawShape(shape, tf1, drawMode);
                    renderer.DrawLine(tf0.position, tf1.position);
                }
            }
            else
            {
                struct TempClosestCallback : public ShapeCastClosestCallback
                {
                    Renderer* r;

                    Transform tf0;
                    Vec2 translation;
                    const Shape* shape;

                    void OnHitClosest(Collider* collider, const Vec2& point, const Vec2& normal, float t)
                    {
                        Renderer::DrawMode drawMode;
                        drawMode.fill = false;
                        drawMode.outline = true;

                        Transform tf1 = tf0;
                        tf1.position += translation * t;

                        r->DrawShape(shape, tf1, drawMode);
                        r->DrawLine(tf0.position, tf1.position);
                        r->DrawPoint(point);
                        r->DrawLine(point, point + normal * 0.1f);
                    }
                } callback;

                callback.r = &renderer;
                callback.translation = translation;
                callback.tf0 = tf0;
                callback.shape = shape;

                bool hit = world->ShapeCastClosest(shape, tf0, translation, &callback);
                if (hit == false)
                {
                    Transform tf1 = tf0;
                    tf1.position += translation;

                    renderer.DrawShape(shape, tf1, drawMode);
                    renderer.DrawLine(tf0.position, tf1.position);
                }
            }
        }
    }

    void UpdateUI() override
    {
        ImGui::SetNextWindowPos({ Window::Get().GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (world->GetBodyCount() > 1)
        {
            if (ImGui::Begin("Shape cast", NULL, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("Translation");
                ImGui::DragFloat2("##Translation", &translation.x, 0.1f);
                ImGui::Checkbox("Closest", &closest);
            }
            ImGui::End();
        }
    }

    ~ShapeCastWorld()
    {
        options.show_contact_normal = false;
        options.show_contact_point = false;
    }

    static Demo* Create(Game& game)
    {
        return new ShapeCastWorld(game);
    }
};

DemoFrame shape_cast_2{ "Shape cast world", ShapeCastWorld::Create };

} // namespace muli