cmake_minimum_required(VERSION 3.13)

# variables
set(PROJECT_NAME cpphysics)
set(CMAKE_CXX_STANDARD 17)

set(WINDOW_TITLE "Physics engine")
set(WINDOW_WIDTH 1280)
set(WINDOW_HEIGHT 720)

project(${PROJECT_NAME})
add_executable(${PROJECT_NAME}
    src/main.cpp src/common.h
    src/window.cpp src/window.h
    src/engine.cpp src/engine.h
)

include(Dependency.cmake)

# 우리 프로젝트에 include / lib 관련 옵션 추가
target_include_directories(${PROJECT_NAME} PUBLIC ${DEP_INCLUDE_DIR})
target_link_directories(${PROJECT_NAME} PUBLIC ${DEP_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_LIBS})

target_compile_definitions(${PROJECT_NAME} PUBLIC
    WINDOW_TITLE="${WINDOW_TITLE}"
    WINDOW_WIDTH=${WINDOW_WIDTH}
    WINDOW_HEIGHT=${WINDOW_HEIGHT}
)

# Dependency들이 먼저 build 될 수 있게 관계 설정
add_dependencies(${PROJECT_NAME} ${DEP_LIST})