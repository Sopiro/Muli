file(GLOB_RECURSE DEMO_HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE DEMO_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "include" FILES ${DEMO_HEADER_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "src" FILES ${DEMO_SOURCE_FILES})

add_executable(demo ${DEMO_SOURCE_FILES} ${DEMO_HEADER_FILES})

target_precompile_headers(demo
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/common.h
)

target_include_directories(demo 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(demo 
    PUBLIC 
        muli glad glfw imgui
)

set_target_properties(demo PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(demo PRIVATE /wd4819)
else()
    # target_compile_options(demo PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX .html)

    target_link_options(demo PRIVATE
        -sWASM=1
        -sALLOW_MEMORY_GROWTH=1
        -sASSERTIONS=1
        -sSTACK_SIZE=50MB
        -sINITIAL_MEMORY=100MB
        -sMAXIMUM_MEMORY=1000MB
        -sUSE_GLFW=3
        -sUSE_WEBGL2=1
        -sMAX_WEBGL_VERSION=2
        -sMIN_WEBGL_VERSION=2
        -O3
        -std=c++20
    )
    
endif()
