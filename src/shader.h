#ifndef __SHADER_H__
#define __SHADER_H__

#include "common.h"

class Shader
{
public:
    virtual ~Shader();

    void Use();

protected:
    Shader(const char* vsCode, const char* fsCode);
    uint32_t shaderHandle{ 0 };
};

#endif