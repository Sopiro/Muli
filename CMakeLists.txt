cmake_minimum_required(VERSION 3.9)

project(spe)

# Variables
set(CMAKE_CXX_STANDARD 17)

option(BUILD_DEMO_PROJECT "Build the demo project" ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

include(Dependency.cmake)

add_subdirectory(src)
target_compile_options(spe PUBLIC /wd4566 /wd4819 /wd4251 /wd4244 /wd4267)

if(BUILD_DEMO_PROJECT)
    set(WINDOW_TITLE "SPE Demo")
    set(WINDOW_WIDTH 1280)
    set(WINDOW_HEIGHT 720)

    add_subdirectory(demo)
    target_compile_options(demo PUBLIC /wd4566 /wd4819 /wd4251 /wd4244 /wd4267)

    # default startup project for Visual Studio
    if(MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT demo)
        set_property(TARGET demo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/demo")
    endif()
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
