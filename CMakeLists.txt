cmake_minimum_required(VERSION 3.13)

# Variables
set(PROJECT_NAME cpphysics)
set(CMAKE_CXX_STANDARD 17)

set(WINDOW_TITLE "Physics engine")
set(WINDOW_WIDTH 1280)
set(WINDOW_HEIGHT 720)

project(${PROJECT_NAME})
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/common.h
    src/window.cpp src/window.h
    src/application.cpp src/application.h
    src/input.cpp src/input.h
    src/game.cpp src/game.h
    src/entity.cpp src/entity.h
    src/camera.cpp src/camera.h

    src/rendering/shader.cpp src/rendering/shader.h
    src/rendering/myshader.cpp src/rendering/myshader.h
    src/rendering/mesh.cpp src/rendering/mesh.h

    src/physics/rigidbody.cpp src/physics/rigidbody.h
    src/physics/polygon.cpp src/physics/polygon.h
    src/physics/circle.cpp src/physics/circle.h
    src/physics/box.cpp src/physics/box.h
    src/physics/settings.h
    src/physics/util.cpp src/physics/util.h
    src/physics/aabb.cpp src/physics/aabb.h
    src/physics/aabbtree.cpp src/physics/aabbtree.h
    src/physics/detection.cpp src/physics/detection.h
    src/physics/simplex.cpp src/physics/simplex.h
    src/physics/polytope.cpp src/physics/polytope.h
    src/physics/edge.cpp src/physics/edge.h
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
