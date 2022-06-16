#pragma once

#include "common.h"

class Shader
{
public:
    static std::unique_ptr<Shader> create(const char* vsCode, const char* fsCode);
    virtual ~Shader();

    void setColor(glm::vec3 _color);
    void setViewMatrix(glm::mat4 _viewMatrix);
    void setProjectionMatrix(glm::mat4 _projMatrix);
    void setModelMatrix(glm::mat4 _modelMatrix);

    void use();

private:
    Shader(uint32_t vertexShader, uint32_t fragmentShader, uint32_t shaderProgram);

    uint32_t vertexShader{ 0 };
    uint32_t fragmentShader{ 0 };
    uint32_t shaderProgram{ 0 };

    // uniforms 
    uint32_t colorLoc;
    glm::vec3 color{ 0.0f };

    uint32_t viewLoc;
    glm::mat4 viewMatrix{ 1.0 };

    uint32_t projLoc;
    glm::mat4 projMatrix{ 1.0 };

    uint32_t modelLoc;
    glm::mat4 modelMatrix{ 1.0 };
};