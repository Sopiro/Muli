#pragma once

#include "common.h"

namespace spe
{
class Shader
{
public:
    virtual ~Shader() noexcept;

    Shader(const Shader&) noexcept = delete;
    Shader& operator=(const Shader&) noexcept = delete;

    Shader(Shader&&) noexcept = delete;
    Shader& operator=(Shader&&) noexcept = delete;

    inline void Use();

protected:
    Shader(const char* vsCode, const char* fsCode);

    std::unordered_map<std::string, uint32_t> uniformMap;
    uint32_t shaderHandle{ 0 };
};

inline void Shader::Use()
{
    glUseProgram(shaderHandle);
}

}