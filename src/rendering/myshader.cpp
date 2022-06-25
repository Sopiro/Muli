#include "myshader.h"    

using namespace spe;

std::unique_ptr<MyShader> MyShader::Create()
{
    return std::unique_ptr<MyShader>(new MyShader);
}

MyShader::MyShader() : Shader(
    // Vertex shader
    R"(
        #version 330 core

        layout (location = 0) in vec3 pos;
        layout (location = 1) in vec2 _texCoord;

        out vec2 texCoord;

        uniform mat4 view;
        uniform mat4 proj;
        uniform mat4 model;

        void main()
        {
           mat4 mvp = proj * view * model;
           gl_Position = mvp * vec4(pos, 1.0);

           texCoord = _texCoord;
        }
    )",
    // Fragment shader
    R"(
        #version 330 core
        
        out vec4 fragColor;

        in vec2 texCoord;

        uniform vec3 color;

        void main()
        {
            fragColor = vec4(color, 1.0f);

            vec2 uv = texCoord - vec2(0.5f, 0.5f);

            // Create outline
            if(max(abs(uv.x), abs(uv.y)) > 0.48)
                fragColor = vec4(0.0f, 0.0f, 0.0f, 1.0f);
        }
    )"
)
{
    uniformMap.insert({"color", glGetUniformLocation(shaderHandle, "color")});
    uniformMap.insert({"model", glGetUniformLocation(shaderHandle, "model")});
    uniformMap.insert({"view", glGetUniformLocation(shaderHandle, "view")});
    uniformMap.insert({"proj", glGetUniformLocation(shaderHandle, "proj")});
}

void MyShader::SetColor(glm::vec3 _color)
{
    color = std::move(_color);
    glUniform3fv(uniformMap["color"], 1, &color.r);
}

void MyShader::SetModelMatrix(glm::mat4 _modelMatrix)
{
    modelMatrix = std::move(_modelMatrix);
    glUniformMatrix4fv(uniformMap["model"], 1, GL_FALSE, glm::value_ptr(modelMatrix));
}

void MyShader::SetViewMatrix(glm::mat4 _viewMatrix)
{
    viewMatrix = std::move(_viewMatrix);
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, glm::value_ptr(viewMatrix));
}

void MyShader::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    projMatrix = std::move(_projMatrix);
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, glm::value_ptr(projMatrix));
}