#include "muli/prismatic_joint.h"
#include "muli/world.h"

namespace muli
{

PrismaticJoint::PrismaticJoint(RigidBody* _bodyA,
                               RigidBody* _bodyB,
                               Vec2 _anchor,
                               Vec2 _dir,
                               const WorldSettings& _settings,
                               float _frequency,
                               float _dampingRatio,
                               float _jointMass)
    : Joint(Joint::Type::JointPrismatic, _bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchorA = MulT(bodyA->GetTransform(), _anchor);
    localAnchorB = MulT(bodyB->GetTransform(), _anchor);

    angleOffset = bodyB->GetAngle() - bodyA->GetAngle();

    if (_dir.Length2() < FLT_EPSILON)
    {
        Vec2 u = MulT(bodyA->GetRotation(), (_bodyB->GetPosition() - _bodyA->GetPosition()).Normalized());
        localYAxis = Cross(1.0f, u);
    }
    else
    {
        localYAxis = Cross(1.0f, _dir.Normalized());
    }
}

void PrismaticJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [-t^t, -(ra + u)×t, t^t, rb×t] // Line
    //     [   0,          -1,   0,    1] // Angle
    // M = (J · M^-1 · J^t)^-1

    ra = bodyA->GetRotation() * localAnchorA;
    rb = bodyB->GetRotation() * localAnchorB;

    Vec2 pa = bodyA->GetPosition() + ra;
    Vec2 pb = bodyB->GetPosition() + rb;
    Vec2 d = pb - pa;

    perp = bodyA->GetRotation() * localYAxis;
    sa = Cross(ra + d, perp);
    sb = Cross(rb, perp);

    Mat2 k;
    k[0][0] = bodyA->invMass + bodyB->invMass + sa * sa * bodyA->invInertia + sb * sb * bodyB->invInertia;
    k[1][0] = sa * bodyA->invInertia + sb * bodyB->invInertia;
    k[0][1] = k[1][0];
    k[1][1] = bodyA->invInertia + bodyB->invInertia;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    float error0 = Dot(d, perp);
    float error1 = bodyB->GetAngle() - bodyA->GetAngle() - angleOffset;

    bias.Set(error0, error1);
    bias *= beta * settings.INV_DT;

    if (settings.WARM_STARTING)
    {
        ApplyImpulse(impulseSum);
    }
}

void PrismaticJoint::SolveVelocityConstraint()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec2 jv;
    jv.x = Dot(perp, bodyB->linearVelocity - bodyA->linearVelocity) + sb * bodyB->angularVelocity - sa * bodyA->angularVelocity;
    jv.y = bodyB->angularVelocity - bodyA->angularVelocity;

    Vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void PrismaticJoint::ApplyImpulse(const Vec2& lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->linearVelocity -= (perp * lambda.x) * bodyA->invMass;
    bodyA->angularVelocity -= (lambda.x * sa + lambda.y) * bodyA->invInertia;
    bodyB->linearVelocity += (perp * lambda.x) * bodyB->invMass;
    bodyB->angularVelocity += (lambda.x * sa + lambda.y) * bodyB->invInertia;
}

} // namespace muli