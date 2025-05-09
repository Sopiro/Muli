#include "demo.h"

namespace muli
{

void CreateRagdoll(World* world, float headX, float headY, float scale, int32 gruop)
{
    // todo: Make body hierarchy and return it
    bool continuous = true;

    CollisionFilter filter;
    filter.group = -gruop;

    float headRadius = 0.3f * scale;

    RigidBody* head = world->CreateCircle(headRadius);
    head->SetContinuous(true);
    head->SetPosition(headX, headY);
    head->SetCollisionFilter(filter);

    float bodyWidth = 0.8f * scale;
    float bodyHeight = 1.4f * scale;
    float neckGap = 0.05f * scale;

    RigidBody* body = world->CreateCapsule(bodyHeight / 2.0f, bodyWidth / 2.0f);
    body->SetPosition(headX, headY - headRadius - bodyHeight / 2.0f);
    body->SetCollisionFilter(filter);
    body->SetContinuous(continuous);

    {
        float angle = DegToRad(5);
        world->CreateRevoluteJoint(body, head, body->GetPosition() + Vec2{ 0.0f, bodyHeight / 2.0f }, 60.0f);
        world->CreateLimitedAngleJoint(body, head, -angle, angle, 60.0f, 1.0f, body->GetMass());
    }

    // Arms
    {
        float armRadius = 0.15f * scale;
        float armLength = 0.8f * scale;
        float bodyArmGap = -1.5f * armRadius;

        float armGap = 0;
        float armStartX = (bodyWidth / 2.0f + armRadius + bodyArmGap);
        float armStartY = (headRadius + neckGap + armRadius);

        RigidBody* rightUpperArm = world->CreateCapsule(
            Vec2{ headX + armStartX, headY - armStartY }, Vec2{ headX + armStartX + armLength, headY - armStartY }, armRadius
        );
        rightUpperArm->SetContinuous(continuous);
        rightUpperArm->SetCollisionFilter(filter);

        RigidBody* rightLowerArm = world->CreateCapsule(
            Vec2{ headX + armStartX + armLength + armGap, headY - armStartY },
            Vec2{ headX + armStartX + armLength + armGap + armLength, headY - armStartY }, armRadius
        );
        rightLowerArm->SetContinuous(continuous);
        rightLowerArm->SetCollisionFilter(filter);

        RigidBody* leftUpperArm = world->CreateCapsule(
            Vec2{ headX - armStartX, headY - armStartY }, Vec2{ headX - armStartX - armLength, headY - armStartY }, armRadius
        );
        leftUpperArm->SetContinuous(continuous);
        leftUpperArm->SetCollisionFilter(filter);

        RigidBody* leftLowerArm = world->CreateCapsule(
            Vec2{ headX - armStartX - armLength - armGap, headY - armStartY },
            Vec2{ headX - armStartX - armLength - armGap - armLength, headY - armStartY }, armRadius
        );
        leftLowerArm->SetContinuous(continuous);
        leftLowerArm->SetCollisionFilter(filter);

        {
            float armFrequency = 60.0f;
            float armDampingRatio = 1.0f;

            float armAngleFrequency = 5.0f;
            float armAngleDampingRatio = 1.0f;

            float minAngle = DegToRad(100.0f);
            float maxAngle = DegToRad(80.0f);

            world->CreateRevoluteJoint(
                body, rightUpperArm, Vec2{ headX + armStartX, headY - armStartY }, armFrequency, armDampingRatio, body->GetMass()
            );
            world->CreateLimitedAngleJoint(
                body, rightUpperArm, -minAngle, maxAngle, armAngleFrequency, armAngleDampingRatio, body->GetMass()
            );

            world->CreateRevoluteJoint(
                rightUpperArm, rightLowerArm, Vec2{ headX + armStartX + armLength + armGap, headY - armStartY }, armFrequency,
                armDampingRatio, rightUpperArm->GetMass()
            );
            world->CreateLimitedAngleJoint(
                rightUpperArm, rightLowerArm, -minAngle, maxAngle, armAngleFrequency, armAngleDampingRatio,
                rightUpperArm->GetMass()
            );

            world->CreateRevoluteJoint(
                body, leftUpperArm, Vec2{ headX - armStartX, headY - armStartY }, armFrequency, armDampingRatio, body->GetMass()
            );
            world->CreateLimitedAngleJoint(
                body, leftUpperArm, -maxAngle, minAngle, armAngleFrequency, armAngleDampingRatio, body->GetMass()
            );

            world->CreateRevoluteJoint(
                leftUpperArm, leftLowerArm, Vec2{ headX - armStartX - armLength - armGap, headY - armStartY }, armFrequency,
                armDampingRatio, leftUpperArm->GetMass()
            );
            world->CreateLimitedAngleJoint(
                leftUpperArm, leftLowerArm, -maxAngle, minAngle, armAngleFrequency, armAngleDampingRatio, leftUpperArm->GetMass()
            );
        }
    }

    // Legs
    {
        float legStartX = 0.25f * scale;
        float legRadius = 0.16f * scale;
        float bodyLegGap = -1.5f * legRadius;
        float legLength = 1.0f * scale;
        float legGap = 0.0f;
        float legStartY = (bodyHeight + headRadius + neckGap + legRadius + bodyLegGap);

        RigidBody* rightUpperLeg = world->CreateCapsule(
            Vec2{ headX + legStartX, headY - legStartY }, Vec2{ headX + legStartX, headY - legStartY - legLength }, legRadius
        );
        rightUpperLeg->SetContinuous(continuous);
        rightUpperLeg->SetCollisionFilter(filter);

        RigidBody* rightLowerLeg = world->CreateCapsule(
            Vec2{ headX + legStartX, headY - legStartY - legLength - legGap },
            Vec2{ headX + legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius
        );
        rightLowerLeg->SetContinuous(continuous);
        rightLowerLeg->SetCollisionFilter(filter);

        RigidBody* leftUpperLeg = world->CreateCapsule(
            Vec2{ headX - legStartX, headY - legStartY }, Vec2{ headX - legStartX, headY - legStartY - legLength }, legRadius
        );
        leftUpperLeg->SetContinuous(continuous);
        leftUpperLeg->SetCollisionFilter(filter);

        RigidBody* leftLowerLeg = world->CreateCapsule(
            Vec2{ headX - legStartX, headY - legStartY - legLength - legGap },
            Vec2{ headX - legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius
        );
        leftLowerLeg->SetContinuous(continuous);
        leftLowerLeg->SetCollisionFilter(filter);

        {
            float LegFrequency = 30.0f;
            float LegDampingRatio = 1.0f;

            float legAngleFrequency = 5.0f;
            float legAngleDampingRatio = 1.0f;

            float minAngle = DegToRad(30.0f);
            float maxAngle = DegToRad(75.0f);

            world->CreateRevoluteJoint(
                body, rightUpperLeg, Vec2{ headX + legStartX, headY - legStartY }, LegFrequency, LegDampingRatio, body->GetMass()
            );
            world->CreateLimitedAngleJoint(
                body, rightUpperLeg, -minAngle, maxAngle, legAngleFrequency, legAngleDampingRatio, body->GetMass()
            );

            world->CreateRevoluteJoint(
                rightUpperLeg, rightLowerLeg, Vec2{ headX + legStartX, headY - legStartY - legLength - legGap }, LegFrequency,
                LegDampingRatio, body->GetMass()
            );
            world->CreateLimitedAngleJoint(
                rightUpperLeg, rightLowerLeg, -minAngle, maxAngle, legAngleFrequency, legAngleDampingRatio,
                rightUpperLeg->GetMass()
            );

            world->CreateRevoluteJoint(
                body, leftUpperLeg, Vec2{ headX - legStartX, headY - legStartY }, LegFrequency, LegDampingRatio, body->GetMass()
            );
            world->CreateLimitedAngleJoint(
                body, leftUpperLeg, -maxAngle, minAngle, legAngleFrequency, legAngleDampingRatio, body->GetMass()
            );

            world->CreateRevoluteJoint(
                leftUpperLeg, leftLowerLeg, Vec2{ headX - legStartX, headY - legStartY - legLength - legGap }, LegFrequency,
                LegDampingRatio, body->GetMass()
            );
            world->CreateLimitedAngleJoint(
                leftUpperLeg, leftLowerLeg, -maxAngle, minAngle, legAngleFrequency, legAngleDampingRatio, leftUpperLeg->GetMass()
            );
        }
    }
}

class Ragdoll : public Demo
{
public:
    Ragdoll(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, identity, RigidBody::static_body);

        CreateRagdoll(world, 0.0f, 5.0f, 1.0f, 1);

        RigidBody* c = world->CreateCircle(0.6f);
        float r = Rand(0.0f, pi);
        Vec2 p{ Cos(r), Sin(r) };
        p *= 8.0f;

        c->SetLinearVelocity(-p * Rand(4.0f, 8.0f) + Vec2{ 0.0f, Rand(5.0f, 15.0f) });
        p.y += 0.5f;
        c->SetPosition(p);

        camera.scale.Set(1.5f);
        camera.position.y = (screenBounds.y * 1.5f) / 2;
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

static int index = register_demo("Ragdoll", Ragdoll::Create, 59);

} // namespace muli
