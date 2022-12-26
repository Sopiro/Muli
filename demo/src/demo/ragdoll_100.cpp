#include "demo.h"
#include "game.h"

namespace muli
{

class Ragdoll100 : public Demo
{
public:
    Ragdoll100(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);
    }

    void CreateRagdoll(float headX, float headY, float scale)
    {
        float motorForce = FLT_MAX;

        float headRadius = 0.3f * scale;

        RigidBody* head = world->CreateCircle(headRadius);
        game.RegisterRenderBody(head);
        head->SetPosition(headX, headY);

        float bodyWidth = 0.8f * scale;
        float bodyHeight = 1.4f * scale;
        float neckGap = 0.05f * scale;

        RigidBody* body = world->CreateBox(bodyWidth, bodyHeight);
        game.RegisterRenderBody(body);
        body->SetPosition(headX, headY - headRadius - bodyHeight / 2.0f - neckGap);

        {
            world->CreateWeldJoint(body, head, 15.0f);
        }

        // Arms
        {
            float bodyArmGap = 0.01f * scale;
            float armRadius = 0.15f * scale;
            float armLength = 0.8f * scale;
            float armGap = armRadius * 2.0f + bodyArmGap;
            float armStartX = (bodyWidth / 2.0f + armRadius + bodyArmGap);
            float armStartY = (headRadius + neckGap + armRadius);

            RigidBody* rightUpperArm = world->CreateCapsule(Vec2{ headX + armStartX, headY - armStartY },
                                                            Vec2{ headX + armStartX + armLength, headY - armStartY }, armRadius);

            RigidBody* rightLowerArm =
                world->CreateCapsule(Vec2{ headX + armStartX + armLength + armGap, headY - armStartY },
                                     Vec2{ headX + armStartX + armLength + armGap + armLength, headY - armStartY }, armRadius);

            RigidBody* leftUpperArm = world->CreateCapsule(Vec2{ headX - armStartX, headY - armStartY },
                                                           Vec2{ headX - armStartX - armLength, headY - armStartY }, armRadius);

            RigidBody* leftLowerArm =
                world->CreateCapsule(Vec2{ headX - armStartX - armLength - armGap, headY - armStartY },
                                     Vec2{ headX - armStartX - armLength - armGap - armLength, headY - armStartY }, armRadius);

            game.RegisterRenderBody(rightUpperArm);
            game.RegisterRenderBody(rightLowerArm);
            game.RegisterRenderBody(leftUpperArm);
            game.RegisterRenderBody(leftLowerArm);

            {
                float armMotorTorque = rightUpperArm->GetMass() * 2.0f * Sqrt(scale);
                float armMotorFrequency = 30.0f;
                float armMotorDampingRatio = 1.0f;

                world->CreateMotorJoint(body, rightUpperArm, Vec2{ headX + armStartX, headY - armStartY }, motorForce,
                                        armMotorTorque, armMotorFrequency, armMotorDampingRatio, body->GetMass());
                world->CreateMotorJoint(rightUpperArm, rightLowerArm,
                                        Vec2{ headX + armStartX + armLength + armGap, headY - armStartY }, motorForce,
                                        armMotorTorque, armMotorFrequency, armMotorDampingRatio, rightUpperArm->GetMass());

                world->CreateMotorJoint(body, leftUpperArm, Vec2{ headX - armStartX, headY - armStartY }, motorForce,
                                        armMotorTorque, armMotorFrequency, armMotorDampingRatio, body->GetMass());
                world->CreateMotorJoint(leftUpperArm, leftLowerArm,
                                        Vec2{ headX - armStartX - armLength - armGap, headY - armStartY }, motorForce,
                                        armMotorTorque, armMotorFrequency, armMotorDampingRatio, leftUpperArm->GetMass());
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

            RigidBody* rightUpperLeg = world->CreateCapsule(Vec2{ headX + legStartX, headY - legStartY },
                                                            Vec2{ headX + legStartX, headY - legStartY - legLength }, legRadius);

            RigidBody* rightLowerLeg =
                world->CreateCapsule(Vec2{ headX + legStartX, headY - legStartY - legLength - legGap },
                                     Vec2{ headX + legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius);

            RigidBody* leftUpperLeg = world->CreateCapsule(Vec2{ headX - legStartX, headY - legStartY },
                                                           Vec2{ headX - legStartX, headY - legStartY - legLength }, legRadius);

            RigidBody* leftLowerLeg =
                world->CreateCapsule(Vec2{ headX - legStartX, headY - legStartY - legLength - legGap },
                                     Vec2{ headX - legStartX, headY - legStartY - legLength - legGap - legLength }, legRadius);

            game.RegisterRenderBody(rightUpperLeg);
            game.RegisterRenderBody(rightLowerLeg);
            game.RegisterRenderBody(leftUpperLeg);
            game.RegisterRenderBody(leftLowerLeg);

            {
                float legMotorTorque = rightUpperLeg->GetMass() * 3.0f * Sqrt(scale);
                float legMotorFrequency = 30.0f;
                float legMotorDampingRatio = 1.0f;

                world->CreateMotorJoint(body, rightUpperLeg, Vec2{ headX + legStartX, headY - legStartY }, motorForce,
                                        legMotorTorque, legMotorFrequency, legMotorDampingRatio, body->GetMass());
                world->CreateMotorJoint(rightUpperLeg, rightLowerLeg,
                                        Vec2{ headX + legStartX, headY - legStartY - legLength - legGap }, motorForce,
                                        legMotorTorque, legMotorFrequency, legMotorDampingRatio, body->GetMass());

                world->CreateMotorJoint(body, leftUpperLeg, Vec2{ headX - legStartX, headY - legStartY }, motorForce,
                                        legMotorTorque, legMotorFrequency, legMotorDampingRatio, body->GetMass());
                world->CreateMotorJoint(leftUpperLeg, leftLowerLeg,
                                        Vec2{ headX - legStartX, headY - legStartY - legLength - legGap }, motorForce,
                                        legMotorTorque, legMotorFrequency, legMotorDampingRatio, body->GetMass());
            }
        }
    }

    int32 count = 0;
    float t0 = 0;

    void Step() override
    {
        Demo::Step();

        float t1 = game.GetTime();
        if (count < 100 && t0 + 0.1f < t1)
        {
            CreateRagdoll(LinearRand(-5.0f, 5.0f), LinearRand(10.0f, 20.0f), 0.3f);
            t0 = t1;
            ++count;
        }
    }

    static Demo* Create(Game& game)
    {
        return new Ragdoll100(game);
    }
};

DemoFrame ragdoll_100{ "Ragdoll 100", Ragdoll100::Create };

} // namespace muli
