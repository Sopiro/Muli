#include "myshader.h"    

std::unique_ptr<MyShader> MyShader::Create()
{
    return std::unique_ptr<MyShader>(new MyShader);
}

MyShader::MyShader() : Shader(
    // Vertex shader
    R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;

        uniform mat4 view;
        uniform mat4 proj;
        uniform mat4 model;

        void main()
        {
           mat4 mvp = proj * view * model;
           gl_Position = mvp * vec4(aPos, 1.0);
        }
    )",
    // Fragment shader
    R"(
        #version 330 core
        out vec4 fragColor;

        uniform vec3 color;

        void main()
        {
           fragColor = vec4(color, 1.0f);
        }
    )"
)
{
    colorLoc = glGetUniformLocation(shaderHandle, "color");
    modelLoc = glGetUniformLocation(shaderHandle, "model");
    viewLoc = glGetUniformLocation(shaderHandle, "view");
    projLoc = glGetUniformLocation(shaderHandle, "proj");
}

MyShader::~MyShader()
{
}

void MyShader::SetColor(glm::vec3 _color)
{
    color = std::move(_color);
    glUniform3fv(colorLoc, 1, &color.r);
}

void MyShader::SetModelMatrix(glm::mat4 _modelMatrix)
{
    modelMatrix = std::move(_modelMatrix);
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));
}

void MyShader::SetViewMatrix(glm::mat4 _viewMatrix)
{
    viewMatrix = std::move(_viewMatrix);
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));
}

void MyShader::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    projMatrix = std::move(_projMatrix);
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projMatrix));
}