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
    src/engine.cpp src/engine.h
    src/game.cpp src/game.h
    src/shader.cpp src/shader.h
    src/myshader.cpp src/myshader.h
    src/mesh.cpp src/mesh.h
    src/entity.cpp src/entity.h
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
