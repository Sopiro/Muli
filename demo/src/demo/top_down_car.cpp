#include "demo.h"

namespace muli
{

// See this article for more: https://www.iforce2d.net/b2dtut/top-down-car

static bool followCam = true;
static bool rotateCam = false;
static bool drawAxis = false;

static float linearDamping = 0.2f;
static float angularDamping = 2.0f;

static float force = 30;
static float torque = 10;

static float friction = 0.4;
static float maxImpulse = 0.5;

static float brake = 10;
static float drag = 0.5f;

struct Wheel
{
    RigidBody* wheel;
    Vec2 forward, normal;

    float force, torque;
    float brake, drag;

    float friction, maxImpulse;

    void Init(
        World* world,
        float scale,
        Transform tf,
        CollisionFilter filter,
        float linearDamping,
        float angularDamping,
        float _force,
        float _friction,
        float _maxImpulse,
        float _brake,
        float _drag
    )
    {
        wheel = world->CreateCapsule(scale, scale, false, tf);
        wheel->SetCollisionFilter(filter);

        wheel->SetLinearDamping(linearDamping);
        wheel->SetAngularDamping(angularDamping);

        force = _force;
        friction = _friction;
        maxImpulse = _maxImpulse;

        brake = _brake;
        drag = _drag;
    }

    void Step()
    {
        const Vec2 up(0, 1);
        const Vec2 right(1, 0);

        forward = Mul(wheel->GetRotation(), up);
        normal = Mul(wheel->GetRotation(), right);

        Vec2 v = wheel->GetLinearVelocity();
        float vf = Dot(v, forward);
        float vn = Dot(v, normal);

        if (Abs(vn) > epsilon)
        {
            Vec2 j = -wheel->GetMass() * friction * vn * normal;
            if (Length(j) > maxImpulse)
            {
                j = Normalize(j) * maxImpulse;
            }
            wheel->ApplyLinearImpulse(wheel->GetPosition(), j, true);
        }

        float av = wheel->GetAngularVelocity();
        if (Abs(av) > epsilon)
        {
            wheel->ApplyAngularImpulse(0.1f * wheel->GetInertia() * -av, true);
        }

        if (Abs(vf) > epsilon)
        {
            float dragForceMagnitude = -drag * vf;
            wheel->ApplyForce(wheel->GetPosition(), dragForceMagnitude * forward, true);
        }
    }

    void UpdateInputAccelerate()
    {
        if (Input::IsKeyDown(GLFW_KEY_W))
        {
            wheel->ApplyForce(wheel->GetPosition(), forward * force, true);
        }

        if (Input::IsKeyDown(GLFW_KEY_S))
        {
            wheel->ApplyForce(wheel->GetPosition(), -forward * force, true);
        }
    }

    void UpdateInputBrake()
    {
        if (Input::IsKeyDown(GLFW_KEY_Q))
        {
            Vec2 brakeForce = -brake * wheel->GetMass() * wheel->GetLinearVelocity();
            wheel->ApplyForce(wheel->GetPosition(), brakeForce, false);
        }
    }

    void DrawDebug(Renderer& r)
    {
        Vec2 p = wheel->GetPosition();

        r.DrawLine(p, p + 0.5f * forward);
        r.DrawLine(p, p + 0.5f * normal);
    }
};

class TopDownCar : public Demo
{
    Wheel wheels[4];
    RigidBody* body;

    MotorJoint* mj[4];

public:
    TopDownCar(Game& game)
        : Demo(game)
    {
        settings.apply_gravity = false;

        CollisionFilter filter;
        filter.group = -1;

        float w = 0.8f;
        float h = 1.4f;

        body = world->CreateBox(w, h);
        body->SetCollisionFilter(filter);

        body->SetLinearDamping(linearDamping);
        body->SetAngularDamping(angularDamping);

        float s = 0.2f;

        // Front wheels
        wheels[0].Init(
            world, s, Transform(Vec2(w / 2, h / 2)), filter, linearDamping, angularDamping, force, friction, maxImpulse, brake,
            drag
        );
        wheels[1].Init(
            world, s, Transform(Vec2(-w / 2, h / 2)), filter, linearDamping, angularDamping, force, friction, maxImpulse, brake,
            drag
        );

        // Rear wheels
        wheels[2].Init(
            world, s, Transform(Vec2(w / 2, -h / 2)), filter, linearDamping, angularDamping, force, friction, maxImpulse, brake,
            drag
        );
        wheels[3].Init(
            world, s, Transform(Vec2(-w / 2, -h / 2)), filter, linearDamping, angularDamping, force, friction, maxImpulse, brake,
            drag
        );

        float mf = -1;
        float fr = -1;
        float dr = 0.1f;
        float jm = body->GetMass();

        for (int32 i = 0; i < 4; ++i)
        {
            mj[i] = world->CreateMotorJoint(body, wheels[i].wheel, wheels[i].wheel->GetPosition(), mf, torque, fr, dr, jm);
        }

        // Obstacles
        float r = 0.3f;
        float spread = 10;
        Vec2 o(10, 0);
        for (int32 i = 0; i < 50; ++i)
        {
            RigidBody* b;
            float random = Rand(0.0f, 3.0f);
            if (random < 1.0f)
            {
                b = world->CreateRandomConvexPolygon(r, 7);
            }
            else if (random < 2.0f)
            {
                b = world->CreateCircle(r);
            }
            else
            {
                b = world->CreateCapsule(r * 1.2f, r * 1.2f / 2.0f);
            }

            b->SetPosition(o + Vec2{ Rand(0.0f, spread) - (spread) / 2.0f, Rand(0.0f, spread) - (spread) / 2.0f });
            b->SetLinearDamping(1);
            b->SetAngularDamping(1);
        }

        camera.position = Vec2::zero;
        camera.scale.Set(2.0f);
    }

    virtual void UpdateInput() override
    {
        Demo::UpdateInput();

        float angle = 35.0f;

        // Front wheel steering
        if (Input::IsKeyDown(GLFW_KEY_A))
        {
            for (int32 i = 0; i < 2; ++i)
            {
                mj[i]->SetAngularOffset(DegToRad(angle));
            }
        }
        else if (Input::IsKeyDown(GLFW_KEY_D))
        {
            for (int32 i = 0; i < 2; ++i)
            {
                mj[i]->SetAngularOffset(-DegToRad(angle));
            }
        }
        else
        {
            for (int32 i = 0; i < 2; ++i)
            {
                mj[i]->SetAngularOffset(0);
            }
        }

        // Rear wheel steering
        if (Input::IsKeyDown(GLFW_KEY_LEFT))
        {
            for (int32 i = 0; i < 2; ++i)
            {
                mj[i + 2]->SetAngularOffset(DegToRad(angle));
            }
        }
        else if (Input::IsKeyDown(GLFW_KEY_RIGHT))
        {
            for (int32 i = 0; i < 2; ++i)
            {
                mj[i + 2]->SetAngularOffset(-DegToRad(angle));
            }
        }
        else
        {
            for (int32 i = 0; i < 2; ++i)
            {
                mj[i + 2]->SetAngularOffset(0);
            }
        }

        // Rear wheel drive
        for (int32 i = 0; i < 2; ++i)
        {
            wheels[i + 2].UpdateInputAccelerate();
        }

        for (int32 i = 0; i < 4; ++i)
        {
            wheels[i].UpdateInputBrake();
        }

        if (followCam)
        {
            camera.position = Lerp(camera.position, body->GetPosition(), 2.0f * dt);
        }
        if (rotateCam)
        {
            camera.rotation = Lerp(camera.rotation, body->GetAngle(), 2.0f * dt);
        }
    }

    virtual void Step() override
    {
        for (int32 i = 0; i < 4; ++i)
        {
            wheels[i].Step();
        }

        Demo::Step();
    }

    virtual void UpdateUI() override
    {
        if (drawAxis)
        {
            for (int32 i = 0; i < 4; ++i)
            {
                wheels[i].DrawDebug(renderer);
            }
        }

        ImGui::SetNextWindowPos({ Window::Get()->GetWindowSize().x - 5, 5 }, ImGuiCond_Once, { 1.0f, 0.0f });

        if (ImGui::Begin("Topdown car", NULL, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Checkbox("Draw axis", &drawAxis);
            ImGui::Checkbox("Follow cam", &followCam);
            ImGui::Checkbox("Rotate cam", &rotateCam);

            ImGui::Text("Linear damping");
            if (ImGui::SliderFloat("##Linear damping", &linearDamping, 0.0f, 5.0f, "%.2f"))
            {
                body->SetLinearDamping(linearDamping);
                for (int32 i = 0; i < 4; ++i)
                {
                    wheels[i].wheel->SetLinearDamping(linearDamping);
                }
            }
            ImGui::Text("Angular damping");
            if (ImGui::SliderFloat("##Angular damping", &angularDamping, 0.0f, 5.0f, "%.2f"))
            {
                body->SetAngularDamping(angularDamping);
                for (int32 i = 0; i < 4; ++i)
                {
                    wheels[i].wheel->SetAngularDamping(angularDamping);
                }
            }

            ImGui::Text("Force");
            if (ImGui::SliderFloat("##Force", &force, 0.0f, 50.0f, "%.2f"))
            {
                for (int32 i = 0; i < 4; ++i)
                {
                    wheels[i].force = force;
                }
            }

            ImGui::Text("Max impulse");
            if (ImGui::SliderFloat("##Max impulse", &maxImpulse, 0.0f, 1.0f, "%.2f"))
            {
                for (int32 i = 0; i < 4; ++i)
                {
                    wheels[i].maxImpulse = maxImpulse;
                }
            }

            ImGui::Text("Side friction");
            if (ImGui::SliderFloat("##Side friction", &friction, 0.0f, 1.0f, "%.2f"))
            {
                for (int32 i = 0; i < 4; ++i)
                {
                    wheels[i].friction = friction;
                }
            }

            ImGui::Text("Torque");
            if (ImGui::SliderFloat("##Torque", &torque, 0.0f, 20.0f, "%.2f"))
            {
                for (int32 i = 0; i < 4; ++i)
                {
                    mj[i]->SetMaxTorque(torque);
                }
            }

            ImGui::Text("Brake");
            if (ImGui::SliderFloat("##Brake", &brake, 0.0f, 20.0f, "%.2f"))
            {
                for (int32 i = 0; i < 4; ++i)
                {
                    wheels[i].brake = brake;
                }
            }

            ImGui::Text("Drag");
            if (ImGui::SliderFloat("##Drag", &drag, 0.0f, 3.0f, "%.2f"))
            {
                for (int32 i = 0; i < 4; ++i)
                {
                    wheels[i].drag = drag;
                }
            }
        }
        ImGui::End();

        ImGui::SetNextWindowPos(
            { Window::Get()->GetWindowSize().x - 5, Window::Get()->GetWindowSize().y - 5 }, ImGuiCond_Always, { 1.0f, 1.0f }
        );
        ImGui::Begin(
            "TopDownCarHelp", NULL,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoBackground
        );
        ImGui::TextColored(ImColor{ 12, 11, 14 }, "WASD and Arrows to control, Q to brake");
        ImGui::End();
    }

    static Demo* Create(Game& game)
    {
        return new TopDownCar(game);
    }
};

static int index = register_demo("Top down car", TopDownCar::Create, 57);

} // namespace muli
