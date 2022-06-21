#ifndef __COMMON_H__
#define __COMMON_H__

#include <spdlog/spdlog.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <imgui.h>

// stds
#include <string>
#include <memory>
#include <chrono>
#include <vector>


// Some useful operator overloading for glm containers
namespace glm
{
    template<typename T, qualifier Q>
    GLM_FUNC_QUALIFIER typename mat<2, 2, T, Q>::col_type operator*(mat<3, 3, T, Q> const& m, typename mat<2, 2, T, Q>::row_type const& v)
    {
        return typename mat<2, 2, T, Q>::col_type
        (
            m[0][0] * v.x + m[1][0] * v.y + m[2][0] * 1,
            m[0][1] * v.x + m[1][1] * v.y + m[2][1] * 1
        );
    }

    template<typename T, qualifier Q>
    GLM_FUNC_QUALIFIER float cross(vec<2, T, Q> const& v1, vec<2, T, Q> const& v2)
    {
        return v1.x * v2.y - v1.y * v2.x;
    }
}

#endif