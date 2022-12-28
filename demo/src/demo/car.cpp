#include "demo.h"

namespace muli
{

class Car : public Demo
{
public:
    RigidBody* body;
    RigidBody* wheel1;
    RigidBody* wheel2;
    MotorJoint* motor1;
    MotorJoint* motor2;

    Car(Game& game)
        : Demo(game)
    {
        RigidBody* ground = world->CreateCapsule(100.0f, 0.2f, true, RigidBody::Type::static_body);

        std::vector<Vec2> vertices;
        vertices.reserve(6);
        vertices.emplace_back(-0.6f, 0.0f);
        vertices.emplace_back(-0.6f, 0.25f);
        vertices.emplace_back(-0.5f, 0.5f);
        vertices.emplace_back(0.1f, 0.5f);
        vertices.emplace_back(0.5f, 0.25f);
        vertices.emplace_back(0.5f, 0.0f);

        body = world->CreatePolygon(vertices, RigidBody::Type::dynamic_body, false);

        wheel1 = world->CreateCircle(0.15f);
        wheel1->SetPosition(-0.4f, 0.0f);
        wheel2 = world->CreateCircle(0.15f);
        wheel2->SetPosition(0.3f, 0.0f);

        body->Translate(0.0f, 5.0f);
        wheel1->Translate(0.0f, 5.0f);
        wheel2->Translate(0.0f, 5.0f);

        CollisionFilter filter;
        filter.bit = 1 << 1;
        filter.mask = ~(1 << 1);

        body->SetCollisionFilter(filter);
        wheel1->SetCollisionFilter(filter);
        wheel2->SetCollisionFilter(filter);

        world->CreateLineJoint(body, wheel1, wheel1->GetPosition(), Vec2{ 0.0f, 1.0f });
        world->CreateLineJoint(body, wheel2, wheel2->GetPosition(), Vec2{ 0.0f, 1.0f });

        float motorMaxForce = 300.0f;
        float motorMaxTorque = 0.0f;
        float motorFrequency = 30.0f;
        float motorDampingRatio = 1.0f;
        float motorMass = wheel1->GetMass();

        motor1 = world->CreateMotorJoint(body, wheel1, wheel1->GetPosition(), motorMaxForce, motorMaxTorque, motorFrequency,
                                         motorDampingRatio, motorMass);
        motor2 = world->CreateMotorJoint(body, wheel2, wheel2->GetPosition(), motorMaxForce, motorMaxTorque, motorFrequency,
                                         motorDampingRatio, motorMass);
    }

    void Step() override
    {
        Demo::Step();

        if (Input::IsKeyDown(GLFW_KEY_D))
        {
            body->Awake();
            motor1->SetMaxTorque(100.0f);
            motor2->SetMaxTorque(100.0f);
            motor1->SetAngularOffset(wheel1->GetAngle() - body->GetAngle() - 0.3f);
            motor2->SetAngularOffset(wheel2->GetAngle() - body->GetAngle() - 0.3f);
        }
        else if (Input::IsKeyDown(GLFW_KEY_A))
        {
            body->Awake();
            motor1->SetMaxTorque(100.0f);
            motor2->SetMaxTorque(100.0f);
            motor1->SetAngularOffset(wheel1->GetAngle() - body->GetAngle() + 0.3f);
            motor2->SetAngularOffset(wheel2->GetAngle() - body->GetAngle() + 0.3f);
        }
        else
        {
            motor1->SetMaxTorque(0.0f);
            motor2->SetMaxTorque(0.0f);
            motor1->SetAngularOffset(wheel1->GetAngle());
            motor2->SetAngularOffset(wheel2->GetAngle());
        }

        if (Input::IsKeyDown(GLFW_KEY_S))
        {
            body->Awake();
            motor1->SetMaxTorque(100.0f);
            motor2->SetMaxTorque(100.0f);
            motor1->SetAngularOffset(wheel1->GetAngle() - body->GetAngle());
            motor2->SetAngularOffset(wheel2->GetAngle() - body->GetAngle());
        }
    }

    static Demo* Create(Game& game)
    {
        return new Car(game);
    }
};

DemoFrame car{ "Car", Car::Create };

} // namespace muli
