#include "muli/weld_joint.h"
#include "muli/world.h"

namespace muli
{

// Revolute joint + Angle joint

WeldJoint::WeldJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
    : Joint(weld_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);

    angleOffset = bodyB->GetAngle() - bodyA->GetAngle();
}

void WeldJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [-I, -skew(ra), I, skew(rb)] // Revolute
    //     [ 0,        -1, 0,        1] // Angle
    // K = (J · M^-1 · J^t)
    // W = K^-1

    // Find k matrix here: https://dyn4j.org/2010/12/weld-constraint/
    ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->GetLocalCenter());
    rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->GetLocalCenter());

    Mat3 k;

    k[0][0] = bodyA->invMass + bodyB->invMass + bodyA->invInertia * ra.y * ra.y + bodyB->invInertia * rb.y * rb.y;
    k[1][0] = -bodyA->invInertia * ra.y * ra.x - bodyB->invInertia * rb.y * rb.x;
    k[0][1] = k[1][0];
    k[1][1] = bodyA->invMass + bodyB->invMass + bodyA->invInertia * ra.x * ra.x + bodyB->invInertia * rb.x * rb.x;

    k[2][0] = -bodyA->invInertia * ra.y - bodyB->invInertia * rb.y;
    k[2][1] = bodyA->invInertia * ra.x + bodyB->invInertia * rb.x;

    k[0][2] = k[2][0];
    k[1][2] = k[2][1];

    k[2][2] = bodyA->invInertia + bodyB->invInertia;

    k[0][0] += gamma;
    k[1][1] += gamma;
    k[2][2] += gamma;

    m = k.GetInverse();

    Vec2 pa = bodyA->motion.c + ra;
    Vec2 pb = bodyB->motion.c + rb;

    Vec2 error01 = pb - pa;
    float error2 = bodyB->motion.a - bodyA->motion.a - angleOffset;

    bias.Set(error01.x, error01.y, error2);
    bias *= beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void WeldJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec3 jv = Vec2{ bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb) -
                    (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)) };
    jv.z = bodyB->angularVelocity - bodyA->angularVelocity;

    Vec3 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void WeldJoint::ApplyImpulse(const Vec3& lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec2 lambda01{ lambda.x, lambda.y };
    float lambda2 = lambda.z;

#if 1 // Shortened
    bodyA->linearVelocity -= lambda01 * bodyA->invMass;
    bodyA->angularVelocity -= (Cross(ra, lambda01) + lambda2) * bodyA->invInertia;
    bodyB->linearVelocity += lambda01 * bodyB->invMass;
    bodyB->angularVelocity += (Cross(rb, lambda01) + lambda2) * bodyB->invInertia;
#else
    // Solve for point-to-point constraint
    bodyA->linearVelocity -= lambda01 * bodyA->invMass;
    bodyA->angularVelocity -= Cross(ra, lambda01) * bodyA->invInertia;
    bodyB->linearVelocity += lambda01 * bodyB->invMass;
    bodyB->angularVelocity += Cross(rb, lambda01) * bodyB->invInertia;

    // Solve for angle constraint
    bodyA->angularVelocity -= lambda2 * bodyA->invInertia;
    bodyB->angularVelocity += lambda2 * bodyB->invInertia;
#endif
}

} // namespace muli