#ifndef __SHADER_H__
#define __SHADER_H__

#include "common.h"

class Shader
{
public:
    static std::unique_ptr<Shader> create(const char* vsCode, const char* fsCode);
    virtual ~Shader();

    void setColor(glm::vec3 color);

    void use();

private:
    Shader(uint32_t vertexShader, uint32_t fragmentShader, uint32_t shaderProgram);

    uint32_t vertexShader{ 0 };
    uint32_t fragmentShader{ 0 };
    uint32_t shaderProgram{ 0 };

    glm::vec3 color;
};

#endif