include(ExternalProject)

# variables
set(DEP_INSTALL_DIR ${PROJECT_BINARY_DIR}/install)
set(DEP_INCLUDE_DIR ${DEP_INSTALL_DIR}/include)
set(DEP_LIB_DIR ${DEP_INSTALL_DIR}/lib)

# glm: math library
ExternalProject_Add(
    dep_glm
    GIT_REPOSITORY "https://github.com/g-truc/glm"
    GIT_TAG "0.9.9.8"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    TEST_COMMAND ""
    INSTALL_COMMAND
    ${CMAKE_COMMAND} -E copy_directory
    ${PROJECT_BINARY_DIR}/dep_glm-prefix/src/dep_glm/glm
    ${DEP_INSTALL_DIR}/include/glm
)

set(DEP_LIST
    dep_glm
)

# demo project dependencies
if(BUILD_DEMO_PROJECT)
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
        -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
        -DGLAD_INSTALL=ON
    )

    # imgui
    add_library(dep_imgui
        extern/imgui/imgui_draw.cpp
        extern/imgui/imgui_tables.cpp
        extern/imgui/imgui_widgets.cpp
        extern/imgui/imgui.cpp
        extern/imgui/imgui_impl_glfw.cpp
        extern/imgui/imgui_impl_opengl3.cpp
        extern/imgui/imgui_demo.cpp
    )
    set(DEP_INCLUDE_DIR ${DEP_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/extern/imgui)

    set(DEP_LIST_DEMO
        dep_spdlog
        dep_glfw
        dep_glad
        dep_glm
        dep_imgui
    )

    set(DEP_LIBS_DEMO
        spdlog
        glfw3
        glad
        dep_imgui
    )

    # imgui need glfw3
    target_include_directories(dep_imgui PRIVATE ${DEP_INCLUDE_DIR})
endif()
