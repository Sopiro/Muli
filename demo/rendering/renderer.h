#pragma once

#include "../common.h"
#include "shader.h"

namespace spe
{
class Renderer
{
public:
    Renderer() = default;
    virtual ~Renderer() noexcept = default;

    Renderer(const Renderer&) noexcept = delete;
    Renderer& operator=(const Renderer&) noexcept = delete;

    Renderer(Renderer&&) noexcept = delete;
    Renderer& operator=(Renderer&&) noexcept = delete;

    virtual void Render() = 0;
};
}