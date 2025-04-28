#include "muli/motor_joint.h"
#include "muli/world.h"

namespace muli
{

MotorJoint::MotorJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    float maxJointForce,
    float maxJointTorque,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(motor_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , linearImpulseSum{ 0.0f }
    , angularImpulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);
    angleOffset = bodyB->GetAngle() - bodyA->GetAngle();

    linearOffset.SetZero();
    angularOffset = 0.0f;

    maxForce = maxJointForce < 0 ? max_value : Clamp<float>(maxJointForce, 0.0f, max_value);
    maxTorque = maxJointTorque < 0 ? max_value : Clamp<float>(maxJointTorque, 0.0f, max_value);
}

void MotorJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [-I, -skew(ra), I, skew(rb)] // Revolute
    //     [ 0,        -1, 0,        1] // Angle
    // W = (J · M^-1 · J^t)^-1

    ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->GetLocalCenter());
    rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->GetLocalCenter());

    Mat2 k0;

    k0[0][0] = bodyA->invMass + bodyB->invMass + bodyA->invInertia * ra.y * ra.y + bodyB->invInertia * rb.y * rb.y;
    k0[1][0] = -bodyA->invInertia * ra.y * ra.x - bodyB->invInertia * rb.y * rb.x;
    k0[0][1] = k0[1][0];
    k0[1][1] = bodyA->invMass + bodyB->invMass + bodyA->invInertia * ra.x * ra.x + bodyB->invInertia * rb.x * rb.x;

    k0[0][0] += gamma;
    k0[1][1] += gamma;

    float k1 = bodyA->invInertia + bodyB->invInertia + gamma;

    m0 = k0.GetInverse();
    m1 = 1.0f / k1;

    Vec2 pa = bodyA->motion.c + ra;
    Vec2 pb = bodyB->motion.c + rb;

    bias0 = pb - pa + linearOffset;
    bias1 = bodyB->motion.a - bodyA->motion.a - angleOffset - angularOffset;

    bias0 *= beta * step.inv_dt;
    bias1 *= beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(linearImpulseSum, angularImpulseSum);
    }
}

void MotorJoint::SolveVelocityConstraints(const Timestep& step)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec2 jv0 =
        (bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) - (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra));
    float jv1 = bodyB->angularVelocity - bodyA->angularVelocity;

    Vec2 lambda0 = m0 * -(jv0 + bias0 + linearImpulseSum * gamma);
    float lambda1 = m1 * -(jv1 + bias1 + angularImpulseSum * gamma);

    // Clamp linear impulse
    {
        float maxLinearImpulse = maxForce * step.dt;
        Vec2 oldLinearImpulse = linearImpulseSum;
        linearImpulseSum += lambda0;

        if (linearImpulseSum.Length2() > maxLinearImpulse * maxLinearImpulse)
        {
            linearImpulseSum.Normalize();
            linearImpulseSum *= maxLinearImpulse;
        }

        lambda0 = linearImpulseSum - oldLinearImpulse;
    }

    // Clamp angular impulse
    {
        float maxAngularImpulse = maxTorque * step.dt;
        float oldAngularImpulse = angularImpulseSum;
        angularImpulseSum += lambda1;

        angularImpulseSum = Clamp<float>(angularImpulseSum, -maxAngularImpulse, maxAngularImpulse);

        lambda1 = angularImpulseSum - oldAngularImpulse;
    }

    ApplyImpulse(lambda0, lambda1);
}

void MotorJoint::ApplyImpulse(const Vec2& lambda0, float lambda1)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

#if 1
    bodyA->linearVelocity -= bodyA->invMass * lambda0;
    bodyA->angularVelocity -= bodyA->invInertia * (Cross(ra, lambda0) + lambda1);
    bodyB->linearVelocity += bodyB->invMass * lambda0;
    bodyB->angularVelocity += bodyB->invInertia * (Cross(rb, lambda0) + lambda1);
#else
    // Solve for point-to-point constraint
    bodyA->linearVelocity -= lambda0 * bodyA->invMass;
    bodyA->angularVelocity -= bodyA->invInertia * Cross(ra, lambda0);
    bodyB->linearVelocity += lambda0 * bodyB->invMass;
    bodyB->angularVelocity += bodyB->invInertia * Cross(rb, lambda0);

    // Solve for angle constraint
    bodyA->angularVelocity -= lambda1 * bodyA->invInertia;
    bodyB->angularVelocity -= lambda1 * bodyB->invInertia;
#endif
}

} // namespace muli