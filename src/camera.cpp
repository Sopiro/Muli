#include "camera.h"

using namespace spe;

Camera::Camera() :
    Entity()
{

}

glm::mat4 Camera::CameraTransform() const
{
    glm::mat4 res = glm::translate(glm::rotate(glm::scale(glm::mat4{ 1.0f }, { 1.0f / scale.x, 1.0f / scale.y, 1.0f }), -rotation, { 0, 0, 1 }), glm::vec3{ -position, -1 });

    return res;
}
