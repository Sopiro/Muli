#include "spe/block_solver.h"
#include "spe/contact_constraint.h"

namespace spe
{

void BlockSolver::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [-n, -ra1 × n, n, rb1 × n
    //      -n, -ra2 × n, n, rb2 × n]
    // K = (J · M^-1 · J^t)
    // M = K^-1

    nc1 = &cc->normalContacts[0];
    nc2 = &cc->normalContacts[1];

    j1 = &nc1->jacobian;
    j2 = &nc2->jacobian;

    k = glm::mat2{ 1.0f };

    k[0][0]
        = cc->bodyA->invMass
        + j1->wa * cc->bodyA->invInertia * j1->wa
        + cc->bodyB->invMass
        + j1->wb * cc->bodyB->invInertia * j1->wb;

    k[1][1]
        = cc->bodyA->invMass
        + j2->wa * cc->bodyA->invInertia * j2->wa
        + cc->bodyB->invMass
        + j2->wb * cc->bodyB->invInertia * j2->wb;

    k[0][1]
        = cc->bodyA->invMass
        + j1->wa * cc->bodyA->invInertia * j2->wa
        + cc->bodyB->invMass
        + j1->wb * cc->bodyB->invInertia * j2->wb;

    k[1][0] = k[0][1];

    assert(glm::determinant(k) != 0);
    m = glm::inverse(k);
}

void BlockSolver::Solve()
{
    // The comments below are copied from Box2D::b2_contact_solver.cpp
    // Check out Box2D: https://box2d.org
    //
    // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
    // Build the mini LCP for this contact patch
    //
    // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
    //  
    // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
    // b = vn0 - velocityBias
    //
    // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
    // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
    // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
    // solution that satisfies the problem is chosen.
    //
    // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
    // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
    //
    // Substitute:
    //
    // x = a + d
    //
    // a := old total impulse
    // x := new total impulse
    // d := incremental impulse 
    //
    // For the current iteration we extend the formula for the incremental impulse
    // to compute the new total impulse:
    //
    // vn = A * d + b
    //     = A * (x - a) + b
    //     = A * x + b - A * a
    //     = A * x + b'
    // b' = b - A * a; 


    glm::vec2 a = { nc1->impulseSum, nc2->impulseSum }; // old total impulse
    assert(a.x >= 0.0f && a.y >= 0.0f);

    // (Velocity constraint) Normal velocity: Jv = 0
    float vn1
        = glm::dot(j1->va, cc->bodyA->linearVelocity)
        + j1->wa * cc->bodyA->angularVelocity
        + glm::dot(j1->vb, cc->bodyB->linearVelocity)
        + j1->wb * cc->bodyB->angularVelocity;

    float vn2
        = glm::dot(j2->va, cc->bodyA->linearVelocity)
        + j2->wa * cc->bodyA->angularVelocity
        + glm::dot(j2->vb, cc->bodyB->linearVelocity)
        + j2->wb * cc->bodyB->angularVelocity;

    glm::vec2 b = { vn1 + nc1->bias, vn2 + nc2->bias };

    // b' = b - K * a
    b = b - (k * a);
    glm::vec2 x{ 0.0f }; // Lambda;

    while (true)
    {
        //
        // Case 1: vn = 0
        // Both constraints are violated
        //
        // 0 = A * x + b'
        //
        // Solve for x:
        //
        // x = - inv(A) * b'
        //
        x = -(m * b);
        if (x.x >= 0.0f && x.y >= 0.0f) break;

        //
        // Case 2: vn1 = 0 and x2 = 0
        // The first constraint is violated and the second constraint is satisfied
        //
        //   0 = a11 * x1 + a12 * 0 + b1' 
        // vn2 = a21 * x1 + a22 * 0 + b2'
        //

        x.x = nc1->effectiveMass * -b.x;
        x.y = 0.0f;
        vn1 = 0.0f;
        vn2 = k[1][0] * x.x + b.y;
        if (x.x >= 0.0f && vn2 >= 0.0f) break;

        //
        // Case 3: vn2 = 0 and x1 = 0
        // The first constraint is satisfied and the second constraint is violated
        //
        // vn1 = a11 * 0 + a12 * x2 + b1' 
        //   0 = a21 * 0 + a22 * x2 + b2'
        //
        x.x = 0.0f;
        x.y = nc2->effectiveMass * -b.y;
        vn1 = k[0][1] * x.y + b.x;
        vn2 = 0.0f;
        if (x.y >= 0.0f && vn1 >= 0.0f) break;

        //
        // Case 4: x1 = 0 and x2 = 0
        // Both constraints are satisfied 
        //
        // vn1 = b1
        // vn2 = b2;
        //
        x.x = 0.0f;
        x.y = 0.0f;
        vn1 = b.x;
        vn2 = b.y;
        if (vn1 >= 0.0f && vn2 >= 0.0f) break;

        // How did you reach here?! something went wrong!
        assert(false);
        break;
    }

    // Get the incremental impulse
    glm::vec2 d = x - a;
    ApplyImpulse(d);

    // Accumulate
    nc1->impulseSum = x.x;
    nc2->impulseSum = x.y;
}

void BlockSolver::ApplyImpulse(const glm::vec2& lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    cc->bodyA->linearVelocity += j1->va * (cc->bodyA->invMass * (lambda.x + lambda.y));
    cc->bodyA->angularVelocity += cc->bodyA->invInertia * (j1->wa * lambda.x + j2->wa * lambda.y);
    cc->bodyB->linearVelocity += j1->vb * (cc->bodyB->invMass * (lambda.x + lambda.y));
    cc->bodyB->angularVelocity += cc->bodyB->invInertia * (j1->wb * lambda.x + j2->wb * lambda.y);
}

}