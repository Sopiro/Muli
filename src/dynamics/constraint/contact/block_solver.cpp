#include "muli/block_solver.h"
#include "muli/contact.h"

namespace muli
{

void BlockSolver::Prepare(Contact* contact)
{
    // Compute Jacobian J and effective mass M
    // J = [-n, -ra1 × n, n, rb1 × n
    //      -n, -ra2 × n, n, rb2 × n]
    // K = (J · M^-1 · J^t)
    // M = K^-1

    c = contact;

    ContactSolver::Jacobian j1 = c->normalSolvers[0].j;
    ContactSolver::Jacobian j2 = c->normalSolvers[1].j;

    float imA = c->b1->invMass;
    float imB = c->b2->invMass;
    float iiA = c->b1->invInertia;
    float iiB = c->b2->invInertia;

    k[0][0] = imA + imB + j1.wa * iiA * j1.wa + j1.wb * iiB * j1.wb;
    k[1][1] = imA + imB + j2.wa * iiA * j2.wa + j2.wb * iiB * j2.wb;
    k[0][1] = imA + imB + j1.wa * iiA * j2.wa + j1.wb * iiB * j2.wb;
    k[1][0] = k[0][1];

    if (k.GetDeterminant() != 0.0f)
    {
        enabled = true;
        m = k.GetInverse();
    }
    else
    {
        enabled = false;
    }
}

// Solve two contact constraints simultaneously
// https://www.gdcvault.com/play/1020603/Physics-for-Game-Programmers-Understanding
void BlockSolver::Solve()
{
    /*
        The comments below are copied from Box2D::b2_contact_solver.cpp
        Check out Box2D: https://box2d.org

        Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
        Build the mini LCP for this contact patch

        vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2

        A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        b = vn0 - velocityBias

        The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
        implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
        vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
        solution that satisfies the problem is chosen.

        In order to acontactount of the acontactumulated impulse 'a' (because of the iterative nature of the solver which only
        requires that the acontactumulated impulse is clamped and not the incremental impulse) we change the impulse variable
        (x_i).

        Substitute:

        x = a + d

        a := old total impulse
        x := new total impulse
        d := incremental impulse

        For the current iteration we extend the formula for the incremental impulse
        to compute the new total impulse:

        vn = A * d + b
            = A * (x - a) + b
            = A * x + b - A * a
            = A * x + b'
        b' = b - A * a;
    */

    ContactSolver* nc1 = &c->normalSolvers[0];
    ContactSolver* nc2 = &c->normalSolvers[1];

    ContactSolver::Jacobian j1 = nc1->j;
    ContactSolver::Jacobian j2 = nc2->j;

    Vec2 a{ nc1->impulse, nc2->impulse }; // old total impulse
    muliAssert(a.x >= 0.0f && a.y >= 0.0f);

    // clang-format off
    // (Velocity constraint) Normal velocity: Jv = 0
    float vn1 = Dot(j1.va, c->b1->linearVelocity)
              + j1.wa * c->b1->angularVelocity
              + Dot(j1.vb, c->b2->linearVelocity)
              + j1.wb * c->b2->angularVelocity;

    float vn2 = Dot(j2.va, c->b1->linearVelocity)
              + j2.wa * c->b1->angularVelocity
              + Dot(j2.vb, c->b2->linearVelocity)
              + j2.wb * c->b2->angularVelocity;
    // clang-format on

    Vec2 b{ vn1 + nc1->bias, vn2 + nc2->bias };

    // b' = b - K * a
    b = b - (k * a);
    Vec2 x{ 0.0f }; // Lambda;

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
    if (x.x >= 0.0f && x.y >= 0.0f)
    {
        goto solved;
    }

    //
    // Case 2: vn1 = 0 and x2 = 0
    // The first constraint is violated and the second constraint is satisfied
    //
    //   0 = a11 * x1 + a12 * 0 + b1'
    // vn2 = a21 * x1 + a22 * 0 + b2'
    //
    x.x = nc1->m * -b.x;
    x.y = 0.0f;
    vn1 = 0.0f;
    vn2 = k[0][1] * x.x + b.y;
    if (x.x >= 0.0f && vn2 >= 0.0f)
    {
        goto solved;
    }

    //
    // Case 3: vn2 = 0 and x1 = 0
    // The first constraint is satisfied and the second constraint is violated
    //
    // vn1 = a11 * 0 + a12 * x2 + b1'
    //   0 = a21 * 0 + a22 * x2 + b2'
    //
    x.x = 0.0f;
    x.y = nc2->m * -b.y;
    vn1 = k[1][0] * x.y + b.x;
    vn2 = 0.0f;
    if (x.y >= 0.0f && vn1 >= 0.0f)
    {
        goto solved;
    }

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
    if (vn1 >= 0.0f && vn2 >= 0.0f)
    {
        goto solved;
    }

// How did you reach here?! something went wrong!
// You can sometimes reach here because of floating point errors :(
#if 0
        muliAssert(false);
#endif

solved:
    // Get the incremental impulse
    Vec2 d = x - a;

    // Apply incremental impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ
    c->b1->linearVelocity += j1.va * (c->b1->invMass * (d.x + d.y));
    c->b1->angularVelocity += c->b1->invInertia * (j1.wa * d.x + j2.wa * d.y);
    c->b2->linearVelocity += j1.vb * (c->b2->invMass * (d.x + d.y));
    c->b2->angularVelocity += c->b2->invInertia * (j1.wb * d.x + j2.wb * d.y);

    // Accumulate
    nc1->impulse = x.x;
    nc2->impulse = x.y;
}

} // namespace muli