cmake_minimum_required(VERSION 3.9)

# Variables
set(CMAKE_CXX_STANDARD 17)

option(BUILD_DEMO_PROJECT "Build the demo project" ON)

project(spe)

include(Dependency.cmake)

add_subdirectory(src)
set_target_properties(spe PROPERTIES COMPILE_FLAGS "/wd4819")

if(BUILD_DEMO_PROJECT)
    set(WINDOW_TITLE "SPE Demo")
    set(WINDOW_WIDTH 1280)
    set(WINDOW_HEIGHT 720)

    add_subdirectory(demo)

    # default startup project for Visual Studio
    if(MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT demo)
        set_property(TARGET demo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/demo")
    endif()

    set_target_properties(demo PROPERTIES COMPILE_FLAGS "/wd4819")
endif()

target_compile_definitions(demo PUBLIC
    WINDOW_TITLE="${WINDOW_TITLE}"
    WINDOW_WIDTH=${WINDOW_WIDTH}
    WINDOW_HEIGHT=${WINDOW_HEIGHT}
)

install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/spe"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)
