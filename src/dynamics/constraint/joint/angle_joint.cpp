#include "muli/angle_joint.h"
#include "muli/world.h"

namespace muli
{

AngleJoint::AngleJoint(
    RigidBody* _bodyA, RigidBody* _bodyB, const WorldSettings& _settings, float _frequency, float _dampingRatio, float _jointMass)
    : Joint(angle_joint, _bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
    , impulseSum{ 0.0f }
{
    angleOffset = bodyB->sweep.a - bodyA->sweep.a;
}

void AngleJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [0 -1 0 1]
    // M = (J · M^-1 · J^t)^-1

    float k = bodyA->invInertia + bodyB->invInertia + gamma;

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    float error = bodyB->sweep.a - bodyA->sweep.a - angleOffset;

    bias = error * beta * settings.INV_DT;

    if (settings.WARM_STARTING)
    {
        ApplyImpulse(impulseSum);
    }
}

void AngleJoint::SolveVelocityConstraint()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = bodyB->angularVelocity - bodyA->angularVelocity;

    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void AngleJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->angularVelocity -= lambda * bodyA->invInertia;
    bodyB->angularVelocity += lambda * bodyB->invInertia;
}

} // namespace muli