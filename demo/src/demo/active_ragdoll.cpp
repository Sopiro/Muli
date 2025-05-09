#include "demo.h"
#include "ragdoll.h"

namespace muli
{

class ActiveRagdoll : public Demo
{
public:
    int32 ragdollGroup = 1;
    Ragdoll ragdoll;
    MotorJoint* joints[Ragdoll::bone_count];

    float bodyTorque = 15;
    float bodyFrequency = 3;
    float bodyDampingRatio = 0.3f;

    float legTorque = 20;
    float legFrequency = 5;
    float legDampingRatio = 0.7f;

    ActiveRagdoll(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::static_body);

        ragdoll = CreateRagdoll(world, { 0.0f, 5.0f }, 1.0f, ragdollGroup);

        {
            RigidBody* body = ragdoll.bones[0].body;
            MotorJoint* j = world->CreateMotorJoint(
                body, ground, ground->GetPosition(), 0.0f, bodyTorque, bodyFrequency, bodyDampingRatio, body->GetMass()
            );

            joints[0] = j;
        }

        // float armTorque = 1;
        // float armFrequency = 1;
        // float armDampingRatio = 0.3f;

        // for (int32 i = Ragdoll::index_upperRightArm; i <= Ragdoll::index_lowerLeftArm; ++i)
        // {
        //     RigidBody* body = ragdoll.bones[i].body;

        //     MotorJoint* j = world->CreateMotorJoint(
        //         ground, body, ground->GetPosition(), 0, armTorque, armFrequency, armDampingRatio, body->GetMass()
        //     );

        //     armJoints.push_back(j);
        // }

        for (int32 i = Ragdoll::index_upperRightLeg; i <= Ragdoll::index_lowerLeftLeg; ++i)
        {
            RigidBody* body = ragdoll.bones[i].body;

            MotorJoint* j = world->CreateMotorJoint(
                body, ground, ground->GetPosition(), 0, legTorque, legFrequency, legDampingRatio, body->GetMass()
            );

            joints[i] = j;
        }

        // Projectile
        {
            RigidBody* c = world->CreateCircle(0.6f);
            float r = Rand(0.0f, pi);
            Vec2 p{ Cos(r), Sin(r) };
            p *= 8.0f;

            c->SetLinearVelocity(-p * Rand(4.0f, 8.0f) + Vec2{ 0.0f, Rand(5.0f, 15.0f) });
            p.y += 0.5f;
            c->SetPosition(p);
        }

        camera.scale.Set(1.5f);
        camera.position.y = (screenBounds.y * 1.5f) / 2;
    }

    void Step()
    {
        RigidBody* body = ragdoll.bones[Ragdoll::index_body].body;

        Vec2 p = body->GetPosition();
        bool found = false;
        Vec2 q;

        float r = 3.0f;

        world->RayCastAny(p, p + Vec2(0, -r), 0.0f, [&](Collider* c, const Vec2& p, const Vec2& normal, float fraction) {
            if (c->GetFilter().group != -ragdollGroup)
            {
                found = true;
                q = p;
                return 0.0f;
            }
            else
            {
                return 1.0f;
            }
        });

        if (found)
        {
            const float pi2 = pi * 2;
            float a = body->GetAngle() + pi;
            float d = pi2 * Floor(a / pi2);

            renderer.DrawPoint(q, Vec4(1, 0, 0, 1));

            joints[Ragdoll::index_body]->SetMaxTorque(bodyTorque);

            float ao = joints[Ragdoll::index_body]->GetAngularOffset();
            joints[Ragdoll::index_body]->SetAngularOffset(-d);

            for (int32 i = Ragdoll::index_upperRightLeg; i <= Ragdoll::index_lowerLeftLeg; ++i)
            {
                joints[i]->SetMaxTorque(legTorque);
                joints[i]->SetAngularOffset(-d);
            }
        }
        else
        {
            joints[Ragdoll::index_body]->SetMaxTorque(0.0f);

            for (int32 i = Ragdoll::index_upperRightLeg; i <= Ragdoll::index_lowerLeftLeg; ++i)
            {
                RigidBody* b = ragdoll.bones[i].body;
                joints[i]->SetMaxTorque(0.0f);
            }
        }
        Demo::Step();
    }

    static Demo* Create(Game& game)
    {
        return new ActiveRagdoll(game);
    }
};

static int index = register_demo("Active ragdoll", ActiveRagdoll::Create, 61);

} // namespace muli
