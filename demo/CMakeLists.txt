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
    CMAKE_COMPILE_WARNING_AS_ERROR ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)