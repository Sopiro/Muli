cmake_minimum_required(VERSION 3.9)

project(spe LANGUAGES CXX VERSION 0.0.1)

# Variables
set(CMAKE_CXX_STANDARD 17)

option(BUILD_DEMO_PROJECT "Build the demo project" ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

include(GNUInstallDirs)

include(Dependency.cmake)

add_subdirectory(src)

if(BUILD_DEMO_PROJECT)
    add_subdirectory(demo)

    # default startup project for Visual Studio
    if(MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT demo)
        set_property(TARGET demo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/demo")
    endif()
endif()

# install glm
install(
    DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/install/include/glm"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# install spe
install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/spe"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
