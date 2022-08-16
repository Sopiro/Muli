# install dependencies for demo project

# spdlog: fast logger library
ExternalProject_Add(
    dep_spdlog
    GIT_REPOSITORY "https://github.com/gabime/spdlog.git"
    GIT_TAG "v1.x"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    TEST_COMMAND ""
    CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DSPDLOG_BUILD_EXAMPLE=OFF
)

# glfw: cross-platform window support
ExternalProject_Add(
    dep_glfw
    GIT_REPOSITORY "https://github.com/glfw/glfw.git"
    GIT_TAG "3.3.7"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    TEST_COMMAND ""
    CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
    -DGLFW_BUILD_EXAMPLES=OFF
    -DGLFW_BUILD_TESTS=OFF
    -DGLFW_BUILD_DOCS=OFF
)

# glad: opengl funciton loader-generator
ExternalProject_Add(
    dep_glad
    GIT_REPOSITORY "https://github.com/Dav1dde/glad"
    GIT_TAG "v0.1.36"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    TEST_COMMAND ""
    CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
    -DGLAD_INSTALL=ON
)

# imgui: GUI library
add_library(dep_imgui
    extern/imgui/imgui_draw.cpp
    extern/imgui/imgui_tables.cpp
    extern/imgui/imgui_widgets.cpp
    extern/imgui/imgui.cpp
    extern/imgui/imgui_impl_glfw.cpp
    extern/imgui/imgui_impl_opengl3.cpp
    extern/imgui/imgui_demo.cpp
)

# imgui need glfw3
target_include_directories(dep_imgui PRIVATE ${DEP_INCLUDE_DIR})

set(DEP_LIST_DEMO
    dep_spdlog
    dep_glfw
    dep_glad
    dep_glm
    dep_imgui
)

set(DEP_LIBS_DEMO
    spdlog$<$<CONFIG:Debug>:d>
    glfw3
    glad
    dep_imgui
)
