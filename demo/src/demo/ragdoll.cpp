#include "demo.h"

namespace muli
{

class Ragdoll : public Demo
{
public:
    Ragdoll(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::Type::static_body);

        // CollisionFilter filter;
        // filter.filter = 1 << 1;
        // filter.mask = ~(1 << 1);

        CreateRagdoll(0.0f, 5.0f, 1.0f);

        RigidBody* c = world->CreateCircle(0.6f);
        float r = Rand(0.0f, pi);
        Vec2 p{ Cos(r), Sin(r) };
        p *= 8.0f;

        c->SetLinearVelocity(-p * Rand(4.0f, 8.0f) + Vec2{ 0.0f, Rand(5.0f, 15.0f) });
        p.y += 0.5f;
        c->SetPosition(p);

        camera.scale = Vec2{ 1.5f };
        camera.position.y += 1.7f;
    }

    void CreateRagdoll(float headX, float headY, float scale)
    {
        float motorForce = max_value;

        float headRadius = 0.3f * scale;

        RigidBody* head = world->CreateCircle(headRadius);
        head->SetPosition(headX, headY);

        float bodyWidth = 0.8f * scale;
        float bodyHeight = 1.4f * scale;
        float neckGap = 0.05f * scale;

        RigidBody* body = world->CreateBox(bodyWidth, bodyHeight);
        body->SetPosition(headX, headY - headRadius - bodyHeight / 2.0f - neckGap);

        {
            world->CreateWeldJoint(body, head, body->GetPosition() + Vec2{ 0.0f, bodyHeight / 2.0f }, 20.0f);
        }

        // Arms
        {
            float bodyArmGap = 0.01f * scale;
            float armRadius = 0.15f * scale;
            float armLength = 0.8f * scale;
            float armGap = armRadius * 2.0f + bodyArmGap;
            float armStartX = (bodyWidth / 2.0f + armRadius + bodyArmGap);
            float armStartY = (headRadius + neckGap + armRadius);

            RigidBody* rightUpperArm = world->CreateCapsule(
                Vec2{ headX + armStartX, headY - armStartY }, Vec2{ headX + armStartX + armLength, headY - armStartY }, armRadius
            );

            RigidBody* rightLowerArm = world->CreateCapsule(
                Vec2{ headX + armStartX + armLength + armGap, headY - armStartY },
                Vec2{ headX + armStartX + armLength + armGap + armLength, headY - armStartY }, armRadius
            );

            RigidBody* leftUpperArm = world->CreateCapsule(
                Vec2{ headX - armStartX, headY - armStartY }, Vec2{ headX - armStartX - armLength, headY - armStartY }, armRadius
            );

            RigidBody* leftLowerArm = world->CreateCapsule(
                Vec2{ headX - armStartX - armLength - armGap, headY - armStartY },
                Vec2{ headX - armStartX - armLength - armGap - armLength, headY - armStartY }, armRadius
            );

            {
                float armMotorTorque = rightUpperArm->GetMass() * 2.0f * Sqrt(scale);
                float armMotorFrequency = 30.0f;
                float armMotorDampingRatio = 1.0f;

                world->CreateMotorJoint(
                    body, rightUpperArm, Vec2{ headX + armStartX, headY - armStartY }, motorForce, armMotorTorque,
                    armMotorFrequency, armMotorDampingRatio, body->GetMass()
                );
                world->CreateMotorJoint(
                    rightUpperArm, rightLowerArm, Vec2{ headX + armStartX + armLength + armGap, headY - armStartY }, motorForce,
                    armMotorTorque, armMotorFrequency, armMotorDampingRatio, rightUpperArm->GetMass()
                );

                world->CreateMotorJoint(
                    body, leftUpperArm, Vec2{ headX - armStartX, headY - armStartY }, motorForce, armMotorTorque,
                    armMotorFrequency, armMotorDampingRatio, body->GetMass()
                );
                world->CreateMotorJoint(
                    leftUpperArm, leftLowerArm, Vec2{ headX - armStartX - armLength - armGap, headY - armStartY }, motorForce,
                    armMotorTorque, armMotorFrequency, armMotorDampingRatio, leftUpperArm->GetMass()
                );
            }
        }

        // Legs
        {
            float bodyLegGap = 0.01f * scale;
            float legStartX = 0.25f * scale;
            float legRadius = 0.16f * scale;
            float legLength = 1.0f * scale;
            float legGap = legRadius * 2.0f + bodyLegGap;
            float legStartY = (bodyHeight + headRadius + neckGap + legRadius + bodyLegGap);

            RigidBody* rightUpperLeg = world->CreateCapsule(
                Vec2{ headX + legStartX, headY - legStartY }, Vec2{ headX + legStartX, headY - legStartY - legLength }, legRadius
            );

            RigidBody* rightLowerLeg = world->CreateCapsule(
                Vec2{ headX + legStartX, headY - legStartY - legLength - legGap },
                Vec2{ headX + legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius
            );

            RigidBody* leftUpperLeg = world->CreateCapsule(
                Vec2{ headX - legStartX, headY - legStartY }, Vec2{ headX - legStartX, headY - legStartY - legLength }, legRadius
            );

            RigidBody* leftLowerLeg = world->CreateCapsule(
                Vec2{ headX - legStartX, headY - legStartY - legLength - legGap },
                Vec2{ headX - legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius
            );

            {
                float legMotorTorque = rightUpperLeg->GetMass() * 3.0f * Sqrt(scale);
                float legMotorFrequency = 30.0f;
                float legMotorDampingRatio = 1.0f;

                world->CreateMotorJoint(
                    body, rightUpperLeg, Vec2{ headX + legStartX, headY - legStartY }, motorForce, legMotorTorque,
                    legMotorFrequency, legMotorDampingRatio, body->GetMass()
                );
                world->CreateMotorJoint(
                    rightUpperLeg, rightLowerLeg, Vec2{ headX + legStartX, headY - legStartY - legLength - legGap }, motorForce,
                    legMotorTorque, legMotorFrequency, legMotorDampingRatio, body->GetMass()
                );

                world->CreateMotorJoint(
                    body, leftUpperLeg, Vec2{ headX - legStartX, headY - legStartY }, motorForce, legMotorTorque,
                    legMotorFrequency, legMotorDampingRatio, body->GetMass()
                );
                world->CreateMotorJoint(
                    leftUpperLeg, leftLowerLeg, Vec2{ headX - legStartX, headY - legStartY - legLength - legGap }, motorForce,
                    legMotorTorque, legMotorFrequency, legMotorDampingRatio, body->GetMass()
                );
            }
        }
    }

    void Step()
    {
        Demo::Step();
    }

    static Demo* Create(Game& game)
    {
        return new Ragdoll(game);
    }
};

static int index = register_demo("Ragdoll", Ragdoll::Create, 39);

} // namespace muli
