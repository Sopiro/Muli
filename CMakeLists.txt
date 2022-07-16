cmake_minimum_required(VERSION 3.9)

# Variables
set(PROJECT_NAME spe)
set(CMAKE_CXX_STANDARD 17)

set(WINDOW_TITLE "Physics engine")
set(WINDOW_WIDTH 1280)
set(WINDOW_HEIGHT 720)

project(${PROJECT_NAME})

include(Dependency.cmake)

add_subdirectory(src)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "/wd4819")
target_compile_definitions(${PROJECT_NAME} PUBLIC
    WINDOW_TITLE="${WINDOW_TITLE}"
    WINDOW_WIDTH=${WINDOW_WIDTH}
    WINDOW_HEIGHT=${WINDOW_HEIGHT}
)

add_dependencies(${PROJECT_NAME} ${DEP_LIST})
