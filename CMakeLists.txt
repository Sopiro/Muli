cmake_minimum_required(VERSION 3.13)

# Variables
set(PROJECT_NAME spe)
set(CMAKE_CXX_STANDARD 17)

set(WINDOW_TITLE "Physics engine")
set(WINDOW_WIDTH 1280)
set(WINDOW_HEIGHT 720)

project(${PROJECT_NAME})
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/window.cpp
    src/application.cpp
    src/input.cpp
    src/game.cpp
    src/entity.cpp
    src/camera.cpp

    src/rendering/shader.cpp
    src/rendering/rigidbody_shader.cpp
    src/rendering/rigidbody_renderer.cpp
    src/rendering/mesh.cpp
    src/rendering/dynamic_shader.cpp
    src/rendering/dynamic_renderer.cpp

    src/physics/world.cpp

    src/physics/rigidbody.cpp
    src/physics/polygon.cpp
    src/physics/circle.cpp
    src/physics/box.cpp
    src/physics/util.cpp
    src/physics/aabb.cpp
    src/physics/aabbtree.cpp
    src/physics/detection.cpp
    src/physics/simplex.cpp
    src/physics/polytope.cpp
    src/physics/edge.cpp

    src/physics/constraint.cpp
    src/physics/contact_constraint.cpp
    src/physics/contact_solver.cpp
    src/physics/block_solver.cpp
)

include(Dependency.cmake)

target_include_directories(${PROJECT_NAME} PUBLIC ${DEP_INCLUDE_DIR})
target_link_directories(${PROJECT_NAME} PUBLIC ${DEP_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_LIBS})

target_compile_definitions(${PROJECT_NAME} PUBLIC
    WINDOW_TITLE="${WINDOW_TITLE}"
    WINDOW_WIDTH=${WINDOW_WIDTH}
    WINDOW_HEIGHT=${WINDOW_HEIGHT}
)

add_dependencies(${PROJECT_NAME} ${DEP_LIST})
