#include "input.h"

namespace spe
{

std::array<bool, GLFW_KEY_LAST + 1> Input::lastKeys{};
std::array<bool, GLFW_KEY_LAST + 1> Input::currKeys{};

std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> Input::lastBtns{};
std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> Input::currBtns{};

glm::vec2 Input::currMousePos{ 0.0f };
glm::vec2 Input::lastMousePos{ 0.0f };
glm::vec2 Input::mouseAcceleration{ 0.0f };

glm::vec2 Input::mouseScroll{ 0.0f };

}