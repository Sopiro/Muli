cmake_minimum_required(VERSION 3.16)

project(muli LANGUAGES C CXX VERSION 0.1.0)

option(MULI_BUILD_DEMO "Build the demo project" ON)

if(MSVC)
    add_compile_options(/MP /wd4819 /wd4996)
    add_definitions(-DUNICODE -D_UNICODE)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

add_subdirectory(src)

if(MULI_BUILD_DEMO)
    add_subdirectory(extern)
    add_subdirectory(demo)

    # default startup project for Visual Studio
    if(MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT demo)
        set_property(TARGET demo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/demo)
    endif()
endif()

# install muli
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/muli
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
