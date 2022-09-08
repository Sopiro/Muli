cmake_minimum_required(VERSION 3.9)

project(spe LANGUAGES CXX VERSION 0.0.1)

option(BUILD_DEMO_PROJECT "Build the demo project" ON)

# compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/MP /wd4819 /wd4996)
    add_definitions(-DUNICODE -D_UNICODE)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(DEP_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/install)
set(DEP_INCLUDE_DIR ${DEP_INSTALL_DIR}/include)
set(DEP_LIB_DIR ${DEP_INSTALL_DIR}/lib)

include(GNUInstallDirs)
include(ExternalProject)

add_subdirectory(src)

if(BUILD_DEMO_PROJECT)
    include(cmake/dependencies.cmake)
    add_subdirectory(demo)

    # default startup project for Visual Studio
    if(MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT demo)
        set_property(TARGET demo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/demo)
    endif()
endif()

# install spe
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/spe
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
