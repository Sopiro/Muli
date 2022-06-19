#ifndef __MYSHADER_H__
#define __MYSHADER_H__

#include "common.h"
#include "shader.h"

class MyShader : public Shader
{
public:
    static std::unique_ptr<MyShader> MyShader::Create();
    virtual ~MyShader();

    void SetColor(glm::vec3 _color);
    void SetViewMatrix(glm::mat4 _viewMatrix);
    void SetProjectionMatrix(glm::mat4 _projMatrix);
    void SetModelMatrix(glm::mat4 _modelMatrix);

private:
    MyShader();

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

#endif