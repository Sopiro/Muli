#pragma once

#include "muli/world.h"

namespace muli
{

struct Bone
{
    int32 parentIndex;
    RigidBody* body;
    RevoluteJoint* revoluteJoint;
    AngleJoint* angleJoint;
};

struct Ragdoll
{
    enum
    {
        index_body = 0,
        index_head = 1,
        index_upperRightArm = 2,
        index_lowerRightArm = 3,
        index_upperLeftArm = 4,
        index_lowerLeftArm = 5,
        index_upperRightLeg = 6,
        index_lowerRightLeg = 7,
        index_upperLeftLeg = 8,
        index_lowerLeftLeg = 9,
        bone_count = 10,
    };

    Bone bones[bone_count];
    float scale;
};

inline Ragdoll CreateRagdoll(World* world, Vec2 headPosition, float scale, int32 gruop)
{
    Ragdoll ragdoll;
    ragdoll.scale = scale;

    bool continuous = true;

    CollisionFilter filter;
    filter.group = -gruop;

    float headRadius = 0.3f * scale;

    float headX = headPosition.x;
    float headY = headPosition.y;

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

    ragdoll.bones[Ragdoll::index_body] = Bone{ -1, body, nullptr, nullptr };

    {
        float angle = DegToRad(5);
        RevoluteJoint* j1 = world->CreateRevoluteJoint(body, head, body->GetPosition() + Vec2{ 0.0f, bodyHeight / 2.0f }, 60.0f);
        AngleJoint* j2 = world->CreateLimitedAngleJoint(body, head, -angle, angle, 60.0f, 1.0f, body->GetMass());

        ragdoll.bones[Ragdoll::index_head] = Bone{ Ragdoll::index_body, head, j1, j2 };
    }

    // Arms
    {
        float armRadius = 0.15f * scale;
        float armLength = 0.8f * scale;
        float bodyArmGap = -1.5f * armRadius;

        float armGap = 0;
        float armStartX = (bodyWidth / 2.0f + armRadius + bodyArmGap);
        float armStartY = (headRadius + neckGap + armRadius);

        RigidBody* upperRightArm = world->CreateCapsule(
            Vec2{ headX + armStartX, headY - armStartY }, Vec2{ headX + armStartX + armLength, headY - armStartY }, armRadius
        );
        upperRightArm->SetContinuous(continuous);
        upperRightArm->SetCollisionFilter(filter);

        RigidBody* lowerRightArm = world->CreateCapsule(
            Vec2{ headX + armStartX + armLength + armGap, headY - armStartY },
            Vec2{ headX + armStartX + armLength + armGap + armLength, headY - armStartY }, armRadius
        );
        lowerRightArm->SetContinuous(continuous);
        lowerRightArm->SetCollisionFilter(filter);

        RigidBody* upperLeftArm = world->CreateCapsule(
            Vec2{ headX - armStartX, headY - armStartY }, Vec2{ headX - armStartX - armLength, headY - armStartY }, armRadius
        );
        upperLeftArm->SetContinuous(continuous);
        upperLeftArm->SetCollisionFilter(filter);

        RigidBody* lowerLeftArm = world->CreateCapsule(
            Vec2{ headX - armStartX - armLength - armGap, headY - armStartY },
            Vec2{ headX - armStartX - armLength - armGap - armLength, headY - armStartY }, armRadius
        );
        lowerLeftArm->SetContinuous(continuous);
        lowerLeftArm->SetCollisionFilter(filter);

        {
            float armFrequency = 60.0f;
            float armDampingRatio = 1.0f;

            float armAngleFrequency = 5.0f;
            float armAngleDampingRatio = 1.0f;

            float minAngle = DegToRad(100.0f);
            float maxAngle = DegToRad(80.0f);

            {

                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    body, upperRightArm, Vec2{ headX + armStartX, headY - armStartY }, armFrequency, armDampingRatio,
                    body->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    body, upperRightArm, -minAngle, maxAngle, armAngleFrequency, armAngleDampingRatio, body->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperRightArm] = Bone{ Ragdoll::index_body, upperRightArm, j1, j2 };
            }

            {
                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    upperRightArm, lowerRightArm, Vec2{ headX + armStartX + armLength + armGap, headY - armStartY }, armFrequency,
                    armDampingRatio, upperRightArm->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    upperRightArm, lowerRightArm, -minAngle, maxAngle, armAngleFrequency, armAngleDampingRatio,
                    upperRightArm->GetMass()
                );

                ragdoll.bones[Ragdoll::index_lowerRightArm] = Bone{ Ragdoll::index_upperRightArm, lowerRightArm, j1, j2 };
            }

            {
                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    body, upperLeftArm, Vec2{ headX - armStartX, headY - armStartY }, armFrequency, armDampingRatio,
                    body->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    body, upperLeftArm, -maxAngle, minAngle, armAngleFrequency, armAngleDampingRatio, body->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperLeftArm] = Bone{ Ragdoll::index_body, upperLeftArm, j1, j2 };
            }

            {
                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    upperLeftArm, lowerLeftArm, Vec2{ headX - armStartX - armLength - armGap, headY - armStartY }, armFrequency,
                    armDampingRatio, upperLeftArm->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    upperLeftArm, lowerLeftArm, -maxAngle, minAngle, armAngleFrequency, armAngleDampingRatio,
                    upperLeftArm->GetMass()
                );

                ragdoll.bones[Ragdoll::index_lowerLeftArm] = Bone{ Ragdoll::index_upperLeftArm, lowerLeftArm, j1, j2 };
            }
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

        RigidBody* upperRightLeg = world->CreateCapsule(
            Vec2{ headX + legStartX, headY - legStartY }, Vec2{ headX + legStartX, headY - legStartY - legLength }, legRadius
        );
        upperRightLeg->SetContinuous(continuous);
        upperRightLeg->SetCollisionFilter(filter);

        RigidBody* lowerRightLeg = world->CreateCapsule(
            Vec2{ headX + legStartX, headY - legStartY - legLength - legGap },
            Vec2{ headX + legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius
        );
        lowerRightLeg->SetContinuous(continuous);
        lowerRightLeg->SetCollisionFilter(filter);

        RigidBody* upperLeftLeg = world->CreateCapsule(
            Vec2{ headX - legStartX, headY - legStartY }, Vec2{ headX - legStartX, headY - legStartY - legLength }, legRadius
        );
        upperLeftLeg->SetContinuous(continuous);
        upperLeftLeg->SetCollisionFilter(filter);

        RigidBody* lowerLeftLeg = world->CreateCapsule(
            Vec2{ headX - legStartX, headY - legStartY - legLength - legGap },
            Vec2{ headX - legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius
        );
        lowerLeftLeg->SetContinuous(continuous);
        lowerLeftLeg->SetCollisionFilter(filter);

        {
            float LegFrequency = 30.0f;
            float LegDampingRatio = 1.0f;

            float legAngleFrequency = 5.0f;
            float legAngleDampingRatio = 1.0f;

            float minAngle = DegToRad(30.0f);
            float maxAngle = DegToRad(75.0f);

            {
                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    body, upperRightLeg, Vec2{ headX + legStartX, headY - legStartY }, LegFrequency, LegDampingRatio,
                    body->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    body, upperRightLeg, -minAngle, maxAngle, legAngleFrequency, legAngleDampingRatio, body->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperRightLeg] = Bone{ Ragdoll::index_body, upperRightLeg, j1, j2 };
            }

            {
                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    upperRightLeg, lowerRightLeg, Vec2{ headX + legStartX, headY - legStartY - legLength - legGap }, LegFrequency,
                    LegDampingRatio, body->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    upperRightLeg, lowerRightLeg, -minAngle, maxAngle, legAngleFrequency, legAngleDampingRatio,
                    upperRightLeg->GetMass()
                );

                ragdoll.bones[Ragdoll::index_lowerRightLeg] = Bone{ Ragdoll::index_upperRightLeg, lowerRightLeg, j1, j2 };
            }

            {
                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    body, upperLeftLeg, Vec2{ headX - legStartX, headY - legStartY }, LegFrequency, LegDampingRatio,
                    body->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    body, upperLeftLeg, -maxAngle, minAngle, legAngleFrequency, legAngleDampingRatio, body->GetMass()
                );

                ragdoll.bones[Ragdoll::index_upperLeftLeg] = Bone{ Ragdoll::index_body, upperLeftLeg, j1, j2 };
            }

            {
                RevoluteJoint* j1 = world->CreateRevoluteJoint(
                    upperLeftLeg, lowerLeftLeg, Vec2{ headX - legStartX, headY - legStartY - legLength - legGap }, LegFrequency,
                    LegDampingRatio, body->GetMass()
                );
                AngleJoint* j2 = world->CreateLimitedAngleJoint(
                    upperLeftLeg, lowerLeftLeg, -maxAngle, minAngle, legAngleFrequency, legAngleDampingRatio,
                    upperLeftLeg->GetMass()
                );

                ragdoll.bones[Ragdoll::index_lowerLeftLeg] = Bone{ Ragdoll::index_upperLeftLeg, lowerLeftLeg, j1, j2 };
            }
        }
    }

    return ragdoll;
}

inline void DeleteRagdoll(World* world, const Ragdoll& ragdoll)
{
    if (world != ragdoll.bones[0].body->GetWorld())
    {
        return;
    }

    for (int32 i = 0; i < Ragdoll::bone_count; ++i)
    {
        world->BufferDestroy(ragdoll.bones[i].body);
    }
}

} // namespace muli
